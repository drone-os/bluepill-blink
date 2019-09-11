//! The root task.

use crate::{
    consts::{PLL_MULT, SYS_CLK, SYS_TICK_FREQ},
    thr,
    thr::ThrsInit,
    Regs,
};
use drone_core::log;
use drone_cortexm::{fib, reg::prelude::*, swo, thr::prelude::*};
use drone_stm32_map::{
    periph::{
        gpio::{periph_gpio_c, GpioC, GpioPortPeriph},
        sys_tick::{periph_sys_tick, SysTickPeriph},
    },
    reg,
};
use futures::prelude::*;

/// An error returned when a receiver has missed too many ticks.
#[derive(Debug)]
pub struct TickOverflow;

/// The root task handler.
#[inline(never)]
pub fn handler(reg: Regs, thr_init: ThrsInit) {
    let thr = thr::init(thr_init);
    let gpio_c = periph_gpio_c!(reg);
    let sys_tick = periph_sys_tick!(reg);

    thr.hard_fault.add_once(|| panic!("Hard Fault"));

    raise_system_frequency(
        reg.flash_acr,
        reg.rcc_cfgr,
        reg.rcc_cir,
        reg.rcc_cr,
        thr.rcc,
    )
    .root_wait();

    beacon(gpio_c, sys_tick, thr.sys_tick)
        .root_wait()
        .expect("beacon fail");

    // Enter a sleep state on ISR exit.
    reg.scb_scr.sleeponexit.set_bit();
}

async fn raise_system_frequency(
    flash_acr: reg::flash::Acr<Srt>,
    rcc_cfgr: reg::rcc::Cfgr<Srt>,
    rcc_cir: reg::rcc::Cir<Srt>,
    rcc_cr: reg::rcc::Cr<Srt>,
    thr_rcc: thr::Rcc,
) {
    thr_rcc.enable_int();
    rcc_cir.modify(|r| r.set_hserdyie().set_pllrdyie());

    // We need to move ownership of `hserdyc` and `hserdyf` into the fiber.
    let reg::rcc::Cir {
        hserdyc, hserdyf, ..
    } = rcc_cir;
    // Attach a listener that will notify us when RCC_CIR_HSERDYF is asserted.
    let hserdy = thr_rcc.add_future(fib::new_fn(move || {
        if hserdyf.read_bit() {
            hserdyc.set_bit();
            fib::Complete(())
        } else {
            fib::Yielded(())
        }
    }));
    // Enable the HSE clock.
    rcc_cr.modify(|r| r.set_hseon());
    // Sleep until RCC_CIR_HSERDYF is asserted.
    hserdy.await;

    // We need to move ownership of `pllrdyc` and `pllrdyf` into the fiber.
    let reg::rcc::Cir {
        pllrdyc, pllrdyf, ..
    } = rcc_cir;
    // Attach a listener that will notify us when RCC_CIR_PLLRDYF is asserted.
    let pllrdy = thr_rcc.add_future(fib::new_fn(move || {
        if pllrdyf.read_bit() {
            pllrdyc.set_bit();
            fib::Complete(())
        } else {
            fib::Yielded(())
        }
    }));
    rcc_cfgr.modify(|r| {
        r.set_pllsrc() // HSE oscillator clock selected as PLL input clock
            .write_pllmul(PLL_MULT - 2) // output frequency = input clock × PLL_MULT
    });
    // Enable the PLL.
    rcc_cr.modify(|r| r.set_pllon());
    // Sleep until RCC_CIR_PLLRDYF is asserted.
    pllrdy.await;

    // Two wait states, if 48 MHz < SYS_CLK <= 72 Mhz.
    flash_acr.modify(|r| r.write_latency(2));

    swo::flush();
    swo::update_prescaler(SYS_CLK / log::baud_rate!() - 1);

    rcc_cfgr.modify(|r| r.write_sw(0b10)); // PLL selected as system clock
}

async fn beacon(
    gpio_c: GpioPortPeriph<GpioC>,
    sys_tick: SysTickPeriph,
    thr_sys_tick: thr::SysTick,
) -> Result<(), TickOverflow> {
    gpio_c.rcc_busenr_gpioen.set_bit(); // GPIO port C clock enable
    gpio_c.gpio_crh.modify(|r| {
        r.write_mode13(0b10) // Output mode, max speed 2 MHz
            .write_cnf13(0b00) // General purpose output push-pull
    });

    // Attach a listener that will notify us on each interrupt trigger.
    let mut tick_stream = thr_sys_tick.add_pulse_try_stream(
        // This closure will be called when a receiver no longer can store the
        // number of ticks since the last stream poll. If this happens, a
        // `TickOverflow` error will be sent over the stream as is final value.
        || Err(TickOverflow),
        // A fiber that will be called on each interrupt trigger. It sends a
        // single tick over the stream.
        fib::new_fn(|| fib::Yielded(Some(1))),
    );
    // Clear the current value of the timer.
    sys_tick.stk_val.store(|r| r.write_current(0));
    // Set the value to load into the `stk_val` register when the counter
    // reaches 0. We set it to the count of SysTick clocks per second divided by
    // 8, so the reload will be triggered each 125 ms.
    sys_tick
        .stk_load
        .store(|r| r.write_reload(SYS_TICK_FREQ / 8));
    sys_tick.stk_ctrl.store(|r| {
        r.set_tickint() // Counting down to 0 triggers the SysTick interrupt
            .set_enable() // Start the counter in a multi-shot way
    });

    // A value cycling from 0 to 7. Full cycle represents a full second.
    let mut counter = 0;
    while let Some(tick) = tick_stream.next().await {
        for _ in 0..tick?.get() {
            // Each full second print a message.
            if counter == 0 {
                println!("sec");
            }
            match counter {
                // On 0's and 250's millisecond pull the pin low.
                0 | 2 => {
                    gpio_c.gpio_bsrr.br13.set_bit();
                }
                // On 125's, 375's, 500's, 625's, 750's, and 875's millisecond
                // pull the pin high.
                _ => {
                    gpio_c.gpio_bsrr.bs13.set_bit();
                }
            }
            counter = (counter + 1) % 8;
        }
    }

    Ok(())
}
