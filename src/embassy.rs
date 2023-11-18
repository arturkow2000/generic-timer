use core::{
    cell::Cell,
    ptr,
    sync::atomic::{AtomicU64, AtomicU8, Ordering},
};

use critical_section::CriticalSection;
use embassy_sync::blocking_mutex::{raw::CriticalSectionRawMutex, CriticalSectionMutex as Mutex};
use embassy_time::{
    driver::{AlarmHandle, Driver},
    time_driver_impl, TICK_HZ,
};

/// We have only one CP15
const ALARM_COUNT: usize = 1;

struct AlarmState {
    timestamp: Cell<u64>,

    // This is really a Option<(fn(*mut ()), *mut ())>
    // but fn pointers aren't allowed in const yet
    callback: Cell<*const ()>,
    ctx: Cell<*mut ()>,
}

unsafe impl Send for AlarmState {}

impl AlarmState {
    const fn new() -> Self {
        Self {
            timestamp: Cell::new(u64::MAX),
            callback: Cell::new(ptr::null()),
            ctx: Cell::new(ptr::null_mut()),
        }
    }
}

const ALARM_STATE_NEW: AlarmState = AlarmState::new();

struct CP15Driver {
    ratio: AtomicU64,
    alarm_count: AtomicU8,
    alarms: Mutex<[AlarmState; ALARM_COUNT]>,
}

impl CP15Driver {
    const fn new() -> Self {
        Self {
            ratio: AtomicU64::new(0),
            alarm_count: AtomicU8::new(0),
            alarms: Mutex::const_new(
                CriticalSectionRawMutex::new(),
                [ALARM_STATE_NEW; ALARM_COUNT],
            ),
        }
    }

    fn init(&self) {
        unsafe {
            if !crate::cp15_is_supported() {
                panic!("CPU does not support CP15 Generic Timer");
            }

            crate::cp15_timer_disable();
            let freq = crate::cp15_read_cntfrq() as u64;
            assert_ne!(freq, 0, "CNTFRQ is zero");
            assert!(
                freq >= TICK_HZ,
                "timer frequency lower than TICK_HZ, this is not supported"
            );

            let ratio = freq / TICK_HZ;
            self.ratio.store(ratio, Ordering::Relaxed);
        }
    }

    fn get_alarm<'a>(&'a self, cs: CriticalSection<'a>, alarm: AlarmHandle) -> &'a AlarmState {
        // safety: we're allowed to assume the AlarmState is created by us, and
        // we never create one that's out of bounds.
        unsafe { self.alarms.borrow(cs).get_unchecked(alarm.id() as usize) }
    }
}

impl Driver for CP15Driver {
    fn now(&self) -> u64 {
        let ticks_raw = unsafe { crate::cp15_read_cntpct() };
        let ratio = self.ratio.load(Ordering::Relaxed);
        ticks_raw / ratio
    }

    unsafe fn allocate_alarm(&self) -> Option<embassy_time::driver::AlarmHandle> {
        let id = self
            .alarm_count
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |x| {
                if x < ALARM_COUNT as u8 {
                    Some(x + 1)
                } else {
                    None
                }
            });

        match id {
            Ok(id) => Some(AlarmHandle::new(id)),
            Err(_) => None,
        }
    }

    fn set_alarm_callback(
        &self,
        alarm: embassy_time::driver::AlarmHandle,
        callback: fn(*mut ()),
        ctx: *mut (),
    ) {
        critical_section::with(|cs| {
            let alarm = self.get_alarm(cs, alarm);

            alarm.callback.set(callback as *const ());
            alarm.ctx.set(ctx);
        })
    }

    fn set_alarm(&self, alarm: embassy_time::driver::AlarmHandle, timestamp: u64) -> bool {
        critical_section::with(|cs| {
            let alarm = self.get_alarm(cs, alarm);
            alarm.timestamp.set(timestamp);

            let ratio = self.ratio.load(Ordering::Relaxed);
            let t = unsafe { crate::cp15_read_cntpct() } / ratio;

            if timestamp <= t {
                return false;
            }

            unsafe { crate::cp15_write_cval(timestamp) };
            true
        })
    }
}

time_driver_impl!(static DRIVER: CP15Driver = CP15Driver::new());

/// Initializes integration with Embassy. After driver is initialized you may
/// use Embassy APIs to schedule tasks and measure time.
///
/// # Restrictions
/// This driver currently does not support interrupts and it will not work
/// properly on executer that sleep. For this driver to work properly one must
/// continuously poll the executor.
///
/// # Safety
/// This driver requires CP15 Generic Timer and will not work on CPU that does
/// not support it. Also, it must be called privileged mode, if called from user
/// mode, exception will be triggered.
pub unsafe fn init() {
    DRIVER.init();
}
