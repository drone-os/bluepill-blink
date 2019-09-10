//! Project constants.

/// HSE crystal frequency.
pub const HSE_FREQ: u32 = 8_000_000;

/// PLL multiplication factor.
pub const PLL_MULT: u32 = 9;

/// System clock frequency.
pub const SYS_CLK: u32 = HSE_FREQ * PLL_MULT;
