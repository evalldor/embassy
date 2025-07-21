use core::char::from_u32;
use core::cmp::min;
use core::marker::PhantomData;
use core::mem::size_of;

use crate::dma::{AnyChannel, Channel};
use crate::pio::{self, program::pio_asm, Common, Config, Direction, PioPin, StateMachine};
use crate::pio::{LoadedProgram, Pin, ShiftDirection};
use crate::spi::{self, Async, Mode};
use crate::{clocks, dma};
use defmt::{error, info};
use embassy_embedded_hal::SetConfig;
use embassy_futures::join::join;
use embassy_hal_internal::Peri;

use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;
use fixed::traits::ToFixed;
use num::{FromPrimitive, ToPrimitive};

pub struct PioSpi<'d, T: pio::Instance, const SM: usize, Word, M: Mode> {
    sm: StateMachine<'d, T, SM>,
    tx_dma: Option<Peri<'d, AnyChannel>>,
    rx_dma: Option<Peri<'d, AnyChannel>>,
    programs: [LoadedProgram<'d, T>; 2],
    clock_pin: Pin<'d, T>,
    current_config: Config<'d, T>,
    phantom: PhantomData<(M, Word)>,
}

impl<'d, T: pio::Instance, const SM: usize, Word, M: Mode> PioSpi<'d, T, SM, Word, M> {
    fn new_inner(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        clk: Peri<'d, impl PioPin + 'd>,
        mosi: Peri<'d, impl PioPin + 'd>,
        miso: Peri<'d, impl PioPin + 'd>,
        tx_dma: Option<Peri<'d, impl Channel>>,
        rx_dma: Option<Peri<'d, impl Channel>>,
        spi_config: &spi::Config,
    ) -> Self {
        let mut clk = pio.make_pio_pin(clk);
        let mut mosi = pio.make_pio_pin(mosi);
        let mut miso = pio.make_pio_pin(miso);

        sm.set_pin_dirs(Direction::Out, &[&clk, &mosi]);
        sm.set_pin_dirs(Direction::In, &[&miso]);

        clk.set_slew_rate(crate::gpio::SlewRate::Slow);
        clk.set_pull(crate::gpio::Pull::None);

        mosi.set_slew_rate(crate::gpio::SlewRate::Slow);
        mosi.set_pull(crate::gpio::Pull::None);

        miso.set_schmitt(true);
        miso.set_pull(crate::gpio::Pull::None);
        miso.set_input_sync_bypass(true);

        let mut config = Config::default();
        config.set_out_pins(&[&mosi]);
        config.set_in_pins(&[&miso]);
        config.shift_out.auto_fill = true;
        config.shift_out.threshold = (size_of::<Word>() * 8) as u8;
        config.shift_out.direction = ShiftDirection::Left;

        config.shift_in.auto_fill = true;
        config.shift_in.threshold = (size_of::<Word>() * 8) as u8;
        config.shift_in.direction = ShiftDirection::Left;

        let prog_cpol0_cpha0 = pio_asm!(
            ".side_set 1
            out pins, 1 side 0 [1]
            in pins, 1  side 1 [1]"
        );

        let prog_cpol0_cpha0 = pio.load_program(&prog_cpol0_cpha0.program);

        let prog_cpol0_cpha1 = pio_asm!(
            ".side_set 1
            out x, 1    side 0
            mov pins, x side 1 [1]
            in pins, 1  side 0"
        );

        let prog_cpol0_cpha1 = pio.load_program(&prog_cpol0_cpha1.program);

        let mut pio_spi = Self {
            sm,
            tx_dma: tx_dma.map_or(None, |c| Some(c.into())),
            rx_dma: rx_dma.map_or(None, |c| Some(c.into())),
            phantom: PhantomData,
            programs: [prog_cpol0_cpha0, prog_cpol0_cpha1],
            clock_pin: clk,
            current_config: config,
        };

        pio_spi.set_spi_config(spi_config);

        pio_spi
    }

    fn set_spi_config(&mut self, config: &spi::Config) {
        let cpha = match config.phase {
            spi::Phase::CaptureOnFirstTransition => 0,
            spi::Phase::CaptureOnSecondTransition => 1,
        };

        let prog = &self.programs[cpha];

        self.clock_pin
            .set_output_inversion(config.polarity == spi::Polarity::IdleHigh);

        self.current_config.use_program(&prog, &[&self.clock_pin]);

        self.current_config.clock_divider = (clocks::clk_sys_freq() / (4 * config.frequency)).to_fixed();

        self.sm.set_enable(false);

        self.sm.set_config(&self.current_config);

        self.sm.clear_fifos();
        self.sm.restart();

        self.sm.set_enable(true);
    }

    fn reset(&mut self) {
        self.sm.clear_fifos();
        self.sm.restart();
    }
}

impl<'d, T: pio::Instance, const SM: usize, Word> PioSpi<'d, T, SM, Word, Async> {
    pub fn new(
        pio: &mut Common<'d, T>,
        sm: StateMachine<'d, T, SM>,
        clk: Peri<'d, impl PioPin + 'd>,
        mosi: Peri<'d, impl PioPin + 'd>,
        miso: Peri<'d, impl PioPin + 'd>,
        tx_dma: Peri<'d, impl Channel>,
        rx_dma: Peri<'d, impl Channel>,
        spi_config: &spi::Config,
    ) -> Self {
        Self::new_inner(pio, sm, clk, mosi, miso, Some(tx_dma), Some(rx_dma), spi_config)
    }
}

impl<'d, T: pio::Instance, const SM: usize, Word, M: Mode> SetConfig for PioSpi<'d, T, SM, Word, M> {
    type Config = spi::Config;
    type ConfigError = ();
    fn set_config(&mut self, config: &Self::Config) -> Result<(), ()> {
        self.set_spi_config(config);

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum PioSpiError {}

impl embedded_hal_1::spi::Error for PioSpiError {
    fn kind(&self) -> embedded_hal_1::spi::ErrorKind {
        embedded_hal_1::spi::ErrorKind::Other
    }
}

impl<'d, T: pio::Instance, const SM: usize, Word, M: Mode> embedded_hal_1::spi::ErrorType
    for PioSpi<'d, T, SM, Word, M>
{
    type Error = PioSpiError;
}

impl<'d, T: pio::Instance, const SM: usize, Word: 'static + Copy + FromPrimitive + ToPrimitive, M: Mode>
    embedded_hal_1::spi::SpiBus<Word> for PioSpi<'d, T, SM, Word, M>
{
    fn read(&mut self, words: &mut [Word]) -> Result<(), Self::Error> {
        for word in words {
            while self.sm.tx().full() {}
            self.sm.tx().push(0);
            while self.sm.rx().empty() {}
            *word = <Word as FromPrimitive>::from_u32(self.sm.rx().pull()).unwrap();
        }

        Ok(())
    }

    fn write(&mut self, words: &[Word]) -> Result<(), Self::Error> {
        let shift_value = 32 - (size_of::<Word>() * 8);

        for word in words {
            while self.sm.tx().full() {}
            self.sm
                .tx()
                .push(<Word as ToPrimitive>::to_u32(word).unwrap() << shift_value);
            while self.sm.rx().empty() {}
            let _ = self.sm.rx().pull();
        }

        Ok(())
    }

    fn transfer(&mut self, read: &mut [Word], write: &[Word]) -> Result<(), Self::Error> {
        let len = read.len().max(write.len());
        let shift_value = 32 - (size_of::<Word>() * 8);

        for i in 0..len {
            let write_val = write
                .get(i)
                .copied()
                .map(|word| <Word as ToPrimitive>::to_u32(&word).unwrap() << shift_value)
                .unwrap_or(0);

            while self.sm.tx().full() {}
            self.sm.tx().push(write_val);

            while self.sm.rx().empty() {}
            let read_val = self.sm.rx().pull();
            if let Some(r) = read.get_mut(i) {
                *r = <Word as FromPrimitive>::from_u32(read_val).unwrap();
            }
        }

        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [Word]) -> Result<(), Self::Error> {
        let shift_value = 32 - (size_of::<Word>() * 8);

        for word in words {
            while self.sm.tx().full() {}
            self.sm
                .tx()
                .push(<Word as ToPrimitive>::to_u32(word).unwrap() << shift_value);
            while self.sm.rx().empty() {}
            *word = <Word as FromPrimitive>::from_u32(self.sm.rx().pull()).unwrap();
        }

        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        while !self.sm.tx().empty() {}
        Ok(())
    }
}

impl<'d, T: pio::Instance, const SM: usize, Word: 'static + Copy + dma::Word> embedded_hal_async::spi::SpiBus<Word>
    for PioSpi<'d, T, SM, Word, Async>
{
    async fn read(&mut self, words: &mut [Word]) -> Result<(), Self::Error> {
        let (rx_fifo, tx_fifo) = self.sm.rx_tx();

        let len = words.len();
        let rx = rx_fifo.dma_pull(self.rx_dma.as_mut().unwrap().reborrow(), words, false);

        let data = 0u32;
        let tx = tx_fifo.dma_push_repeated(self.tx_dma.as_mut().unwrap().reborrow(), &data, len, false);

        join(rx, tx).await;

        Ok(())
    }

    async fn write(&mut self, words: &[Word]) -> Result<(), Self::Error> {
        self.sm.set_autopush(false);

        self.sm
            .tx()
            .dma_push(self.tx_dma.as_mut().unwrap().reborrow(), words, false)
            .await;

        // Data may still be in tx buffer. Wait until state machine is finished

        // Clear stalled flag
        let _ = self.sm.tx().stalled();

        // Wait
        while !self.sm.tx().stalled() {}

        self.sm.restart(); // Clear ISR counter
        self.sm.set_autopush(true);

        Ok(())
    }

    async fn transfer(&mut self, read: &mut [Word], write: &[Word]) -> Result<(), Self::Error> {
        let read_len = read.len();
        let write_len = write.len();

        let num = min(read_len, write_len);

        let (rx_fifo, tx_fifo) = self.sm.rx_tx();

        let rx = rx_fifo.dma_pull(self.rx_dma.as_mut().unwrap().reborrow(), &mut read[..num], false);

        let tx = tx_fifo.dma_push(self.tx_dma.as_mut().unwrap().reborrow(), &write[..num], false);

        join(rx, tx).await;

        if write_len > read_len {
            self.write(&write[num..]).await?;
        } else if read_len > write_len {
            let rx = rx_fifo.dma_pull(self.rx_dma.as_mut().unwrap().reborrow(), &mut read[num..], false);

            let data = 0u32;

            let tx = tx_fifo.dma_push_repeated(
                self.tx_dma.as_mut().unwrap().reborrow(),
                &data,
                read_len - write_len,
                false,
            );

            join(rx, tx).await;
        }

        Ok(())
    }

    async fn transfer_in_place(&mut self, words: &mut [Word]) -> Result<(), Self::Error> {
        let (rx_fifo, tx_fifo) = self.sm.rx_tx();

        let txwords = unsafe {
            // We read and write to the same array, so we trick the borrow
            // checker here
            &*(words as *const [Word])
        };

        let rx = rx_fifo.dma_pull(self.rx_dma.as_mut().unwrap().reborrow(), words, false);
        let tx = tx_fifo.dma_push(self.tx_dma.as_mut().unwrap().reborrow(), txwords, false);

        join(rx, tx).await;

        Ok(())
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}
