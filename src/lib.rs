#![no_std]

#[allow(unused_imports)]
use core::arch::asm;

#[cfg(target_arch = "aarch64")]
use aarch64_cpu::registers::{
    Readable, Writeable, CNTFRQ_EL0, CNTPCT_EL0, CNTP_CTL_EL0, CNTP_CVAL_EL0, CNTP_TVAL_EL0,
};

#[cfg(feature = "embassy_cp15")]
pub mod embassy;

/// Checks whether the calling CPU supports CP15 Generic Timer.
#[inline]
pub unsafe fn cp15_is_supported() -> bool {
    // TODO: can this be called from user mode?
    #[cfg(target_arch = "arm")]
    {
        let mut val: u32;
        asm!("mrc p15, 0, {}, c0, c1, 1", out(reg) val, options(nostack, nomem));
        val >>= 16;
        val &= 0xf;
        val != 0
    }
    #[cfg(target_arch = "aarch64")]
    {
        let mut val: u64;
        asm!("mrs {}, id_pfr1_el1", out(reg) val, options(nostack, nomem));
        val >>= 16;
        val &= 0xf;
        val != 0
    }
    #[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
    unimplemented!()
}

/// Reads timer frequency from CNTFRQ register. CNTFRQ register is undefined
/// after reset, secure software must initialize this register with a correct
/// frequency.
///
/// # Safety
/// Secure firmware must initialize CNTFRQ prior to call to this function,
/// otherwise return value is undefined. This function may trigger undefined
/// instruction exception if called from usermode (behaviour depends on CNTKCTL
/// register), or if CPU does not support CP15 timer.
///
/// # Return value
/// Returns CP15 timer frequency in Hz.
#[inline]
pub unsafe fn cp15_read_cntfrq() -> u32 {
    #[cfg(target_arch = "arm")]
    {
        let val: u32;
        asm!("mrc p15, 0, {}, c14, c0, 0", out(reg) val, options(nostack, nomem));
        val
    }
    #[cfg(target_arch = "aarch64")]
    {
        CNTFRQ_EL0.get() as u32
    }
    #[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
    unimplemented!()
}

/// Write CP15 timer frequency to CNTFRQ register. Please note that this does
/// NOT change the frequency of the timer, it only stores frquency the timer is
/// running at in a register that can be read non-secure firmware.
///
/// # Safety
/// This must be called from Secure World and in privileged mode or it will
/// trigger undefined instruction exception.
#[inline]
pub unsafe fn cp15_write_cntfrq(freq: u32) {
    #[cfg(target_arch = "arm")]
    asm!("mcr p15, 0, {}, c14, c0, 0", in(reg) freq, options(nostack, nomem));

    #[cfg(target_arch = "aarch64")]
    asm!("msr cntfrq_el0, {}", in(reg) (freq as u64), options(nostack, nomem));

    #[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
    {
        let _ = freq;
        unimplemented!()
    }
}

/// Reads CNTPCT register which holds number of clock ticks since boot.
///
/// # Safety
/// If called on a CPU that does not support CP15 this will trigger undefined
/// instruction exception.
/// If called from unprivileged mode and CNTKCTL flags don't allow unprivileged
/// access to CP15 this will trigger undefined instruction exception.
#[inline]
pub unsafe fn cp15_read_cntpct() -> u64 {
    #[cfg(target_arch = "arm")]
    {
        let low: u32;
        let high: u32;
        asm!("mrrc p15, 0, {}, {}, c14", out(reg) low, out(reg) high, options(nostack, nomem));
        low as u64 | ((high as u64) << 32)
    }

    #[cfg(target_arch = "aarch64")]
    {
        CNTPCT_EL0.get()
    }

    #[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
    unimplemented!()
}

/// Write CNTP_TVAL register. This is usually used to configure timer
/// interrupt. After timer is started CNTP_TVAL is decreased on each clock tick
/// and when it reaches 0 an interrupt is fired. Should be called only with
/// disabled timer or the timer may be behave unpredictably.
///
/// # Safety
/// If called on a CPU that does not support CP15 this will trigger undefined
/// instruction exception.
#[inline]
pub unsafe fn cp15_write_tval(ticks: i32) {
    #[cfg(target_arch = "arm")]
    asm!("mcr p15, 0, {}, c14, c2, 0", in(reg) ticks, options(nostack, nomem));

    #[cfg(target_arch = "aarch64")]
    CNTP_TVAL_EL0.set(ticks as u64);

    #[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
    {
        let _ = ticks;
        unimplemented!()
    }
}

/// Write CNTP_CVAL register. This is usually used to configure timer interrupt.
/// When CNTPCT contents become bigger than CNTP_CVAL an interrupt is fired.
///
/// # Safety
/// If called on a CPU that does not support CP15 this will trigger undefined
/// instruction exception.
#[inline]
pub unsafe fn cp15_write_cval(ticks: u64) {
    #[cfg(target_arch = "arm")]
    {
        let ticks_low = ticks as u32;
        let ticks_high = (ticks >> 32) as u32;
        asm!("mcrr p15, 2, {}, {}, c14", in(reg) ticks_low, in(reg) ticks_high, options(nostack, nomem));
    }
    #[cfg(target_arch = "aarch64")]
    CNTP_CVAL_EL0.set(ticks);
    #[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
    {
        let _ = ticks;
        unimplemented!()
    }
}

/// Disable CP15 timer.
///
/// # Safety
/// If called on a CPU that does not support CP15 this will trigger undefined
/// instruction exception.
#[inline]
pub unsafe fn cp15_timer_disable() {
    #[cfg(target_arch = "arm")]
    asm!("mcr p15, 0, {}, c14, c2, 1", in(reg) 0, options(nostack, nomem));
    #[cfg(target_arch = "aarch64")]
    CNTP_CTL_EL0.write(CNTP_CTL_EL0::ENABLE::CLEAR);
    #[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
    unimplemented!()
}

/// Enable CP15 timer.
///
/// # Safety
/// If called on a CPU that does not support CP15 this will trigger undefined
/// instruction exception.
#[inline]
pub unsafe fn cp15_timer_enable(irq_enable: bool) {
    #[cfg(target_arch = "arm")]
    {
        let mut value = 1;
        if !irq_enable {
            value |= 2;
        }

        asm!("mcr p15, 0, {}, c14, c2, 1", in(reg) value, options(nostack, nomem));
    }

    #[cfg(target_arch = "aarch64")]
    {
        CNTP_CTL_EL0.write(
            CNTP_CTL_EL0::ENABLE::SET
                + if irq_enable {
                    CNTP_CTL_EL0::IMASK::SET
                } else {
                    CNTP_CTL_EL0::IMASK::CLEAR
                },
        )
    }

    #[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
    {
        let _ = irq_enable;
        unimplemented!()
    }
}
