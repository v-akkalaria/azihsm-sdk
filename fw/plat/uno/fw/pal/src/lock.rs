// Copyright (c) Microsoft Corporation.
// Licensed under the MIT License.

//! [`HsmPartitionLock`] implementation for the Uno PAL.
//!
//! State-mutating DDI handlers on the same partition must be serialized:
//! each handler runs `check_fail_fast` (nonce, credential-set,
//! provisioned) and then progresses through several `.await` points
//! before committing partition state.  Without a lock, `poll_io` spawns
//! up to 32 concurrent `handle_io` tasks — so multiple handler futures
//! for the same partition can interleave at every yield point, pass
//! their fail-fast checks concurrently, and all race to commit,
//! producing multi-winner behaviour that the write-once state fields
//! alone cannot prevent.
//!
//! We wrap each partition in a per-partition [`embassy_sync::mutex::Mutex`]
//! held via [`NoopRawMutex`] — correct for Embassy's single-threaded
//! executor, where the only contention is between cooperatively
//! scheduled async tasks.  Mirrors the std PAL reference
//! (`fw/plat/std/pal/src/part_lock.rs`).

#![allow(unsafe_code)]

use azihsm_fw_hsm_pal_traits::HsmError;
use azihsm_fw_hsm_pal_traits::HsmIo;
use azihsm_fw_hsm_pal_traits::HsmPartitionLock;
use azihsm_fw_hsm_pal_traits::HsmResult;
use azihsm_fw_uno_drivers_part_store::NUM_PARTITIONS;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::mutex::MutexGuard;

use crate::UnoHsmPal;

/// Per-partition async mutex table serializing state-mutating DDI
/// handlers, one slot per partition id `0..NUM_PARTITIONS`.
///
/// `NoopRawMutex` is correct here: Embassy runs single-threaded on the
/// Uno HSM core and the only contention is between cooperatively
/// scheduled async tasks — no ISR ever calls `partition_lock`.  The
/// `Sync` requirement of `static` is satisfied by the newtype below;
/// its unsafe impl mirrors the single-core, no-ISR contract the Uno
/// PAL already relies on for other statics (see `SingleCell`).
struct PartLocks([Mutex<NoopRawMutex, ()>; NUM_PARTITIONS]);

// SAFETY: Uno is single-core Cortex-M7 with Embassy cooperative
// scheduling.  No ISR accesses these mutexes; the only contention is
// between async tasks on one executor, which `NoopRawMutex` correctly
// serializes.  This matches the `unsafe impl Sync` used by the crate's
// own `SingleCell` primitive for the same reason.
unsafe impl Sync for PartLocks {}

static PART_LOCKS: PartLocks = PartLocks([const { Mutex::new(()) }; NUM_PARTITIONS]);

impl HsmPartitionLock for UnoHsmPal {
    type PartitionGuard<'a> = MutexGuard<'a, NoopRawMutex, ()>;

    /// Acquire the per-partition async mutex for `io.pid()`.
    ///
    /// # Errors
    /// - [`HsmError::InvalidArg`] — `io.pid()` is out of range for the
    ///   configured [`NUM_PARTITIONS`] table.
    async fn partition_lock(&self, io: &impl HsmIo) -> HsmResult<Self::PartitionGuard<'_>> {
        let idx = usize::from(u8::from(io.pid()));
        let slot = PART_LOCKS.0.get(idx).ok_or(HsmError::InvalidArg)?;
        Ok(slot.lock().await)
    }
}
