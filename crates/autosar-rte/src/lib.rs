//! Runtime Environment-style abstraction for the wiper system.
//!
//! # Communication semantics (AUTOSAR RTE mapping)
//!
//! | AUTOSAR concept | This crate |
//! |---|---|
//! | `Rte_Write_<port>_<elem>` | [`port::RteSender::Rte_Write`] |
//! | `Rte_Read_<port>_<elem>` | [`port::RteReceiver::Rte_Read`] |
//! | sender/receiver, non-queued (last-is-best) | [`port::NonQueuedPort`] + sender/receiver |
//! | sender/receiver, queued (FIFO) | [`port::QueuedPort`] with const capacity |
//! | data element invalidation | [`port::RteSender::Rte_Invalidate`] / [`port::ReadStatus::Invalid`] |
//! | timing / send timeout ("alive supervision") | staleness [`port::RteReceiver`] + [`port::ReadStatus::Timeout`] |
//! | client/server port | [`cs::ClientServerPort`] |
//! | mode switch group (mode manager / user) | [`mode::ModeSwitchPort`] |
//! | runnable entity | [`runnable::Runnable`] + [`runnable::Trigger`] |
//!
//! Determinism rules: all time flows in as an explicit `now_ms` argument; the crate
//! never reads a clock. No heap allocation (`no_std`), no `unsafe`.

#![no_std]
#![forbid(unsafe_code)]

pub mod cs;
pub mod mode;
pub mod port;
pub mod runnable;
