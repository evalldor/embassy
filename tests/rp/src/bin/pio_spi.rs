//! Make sure to connect GPIO pins 3 (`PIN_3`) and 4 (`PIN_4`) together
//! to run this test.
//!
#![no_std]
#![no_main]
#[cfg(feature = "rp2040")]
teleprobe_meta::target!(b"rpi-pico");
#[cfg(feature = "rp235xb")]
teleprobe_meta::target!(b"pimoroni-pico-plus-2");

use defmt::{assert_eq, *};
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio_programs::spi::PioSpi;
use embassy_rp::spi::{Async, Config, Phase, Polarity};
use {defmt_rtt as _, panic_probe as _};

use embassy_rp::pio::{InterruptHandler, Pio};
bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});
use embassy_embedded_hal::SetConfig;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Hello World!");

    let clk = p.PIN_10;
    let mosi = p.PIN_11;
    let miso = p.PIN_12;

    let Pio { mut common, sm0, .. } = Pio::new(p.PIO0, Irqs);

    let mut config = Config::default();

    let mut spi: PioSpi<'static, PIO0, 0, u8, Async> =
        PioSpi::new(&mut common, sm0, clk, mosi, miso, p.DMA_CH0, p.DMA_CH1, &config);

    config.frequency = 1_000_000;
    config.phase = Phase::CaptureOnFirstTransition;
    config.polarity = Polarity::IdleLow;

    let _ = spi.set_config(&config);
    test_async(&mut spi).await;
    test_blocking(&mut spi);

    config.phase = Phase::CaptureOnSecondTransition;
    config.polarity = Polarity::IdleLow;

    let _ = spi.set_config(&config);
    test_async(&mut spi).await;
    test_blocking(&mut spi);

    config.phase = Phase::CaptureOnFirstTransition;
    config.polarity = Polarity::IdleHigh;

    let _ = spi.set_config(&config);
    test_async(&mut spi).await;
    test_blocking(&mut spi);

    config.phase = Phase::CaptureOnSecondTransition;
    config.polarity = Polarity::IdleHigh;

    let _ = spi.set_config(&config);
    test_async(&mut spi).await;
    test_blocking(&mut spi);

    // Slow speed
    config.frequency = 1_000;
    config.phase = Phase::CaptureOnFirstTransition;
    config.polarity = Polarity::IdleLow;

    let _ = spi.set_config(&config);
    test_async(&mut spi).await;
    test_blocking(&mut spi);

    config.frequency = 10_000;

    let _ = spi.set_config(&config);
    test_async(&mut spi).await;
    test_blocking(&mut spi);

    config.frequency = 100_000;

    let _ = spi.set_config(&config);
    test_async(&mut spi).await;
    test_blocking(&mut spi);

    config.frequency = 300_000;

    let _ = spi.set_config(&config);
    test_async(&mut spi).await;
    test_blocking(&mut spi);

    info!("Test OK");
    cortex_m::asm::bkpt();
}

async fn test_async<SPI: embedded_hal_async::spi::SpiBus<u8>>(spi: &mut SPI) {
    // equal rx & tx buffers
    {
        let tx_buf = [1_u8, 2, 3, 4, 5, 6];
        let mut rx_buf = [0_u8; 6];
        spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
        assert_eq!(rx_buf, tx_buf);
    }

    // tx > rx buffer
    {
        let tx_buf = [7_u8, 8, 9, 10, 11, 12];

        let mut rx_buf = [0_u8; 3];
        spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
        assert_eq!(rx_buf, tx_buf[..3]);

        defmt::info!("tx > rx buffer - OK");
    }

    // we make sure to that clearing FIFO works after the uneven buffers

    // equal rx & tx buffers
    {
        let tx_buf = [13_u8, 14, 15, 16, 17, 18];
        let mut rx_buf = [0_u8; 6];
        spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();
        assert_eq!(rx_buf, tx_buf);

        defmt::info!("buffer rx length == tx length - OK");
    }

    // rx > tx buffer
    {
        let tx_buf = [19_u8, 20, 21];
        let mut rx_buf = [0_u8; 6];

        // we should have written dummy data to tx buffer to sync clock.
        spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();

        assert_eq!(
            rx_buf[..3],
            tx_buf,
            "only the first 3 TX bytes should have been received in the RX buffer"
        );
        assert_eq!(rx_buf[3..], [0, 0, 0], "the rest of the RX bytes should be empty");
        defmt::info!("buffer rx length > tx length - OK");
    }

    // equal rx & tx buffers
    {
        let tx_buf = [22_u8, 23, 24, 25, 26, 27];
        let mut rx_buf = [0_u8; 6];
        spi.transfer(&mut rx_buf, &tx_buf).await.unwrap();

        assert_eq!(rx_buf, tx_buf);
        defmt::info!("buffer rx length = tx length - OK");
    }
}

fn test_blocking<SPI: embedded_hal_1::spi::SpiBus<u8>>(spi: &mut SPI) {
    // equal rx & tx buffers
    {
        let tx_buf = [1_u8, 2, 3, 4, 5, 6];
        let mut rx_buf = [0_u8; 6];
        spi.transfer(&mut rx_buf, &tx_buf).unwrap();
        assert_eq!(rx_buf, tx_buf);
    }

    // tx > rx buffer
    {
        let tx_buf = [7_u8, 8, 9, 10, 11, 12];

        let mut rx_buf = [0_u8; 3];
        spi.transfer(&mut rx_buf, &tx_buf).unwrap();
        assert_eq!(rx_buf, tx_buf[..3]);

        defmt::info!("tx > rx buffer - OK");
    }

    // we make sure to that clearing FIFO works after the uneven buffers

    // equal rx & tx buffers
    {
        let tx_buf = [13_u8, 14, 15, 16, 17, 18];
        let mut rx_buf = [0_u8; 6];
        spi.transfer(&mut rx_buf, &tx_buf).unwrap();
        assert_eq!(rx_buf, tx_buf);

        defmt::info!("buffer rx length == tx length - OK");
    }

    // rx > tx buffer
    {
        let tx_buf = [19_u8, 20, 21];
        let mut rx_buf = [0_u8; 6];

        // we should have written dummy data to tx buffer to sync clock.
        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(
            rx_buf[..3],
            tx_buf,
            "only the first 3 TX bytes should have been received in the RX buffer"
        );
        assert_eq!(rx_buf[3..], [0, 0, 0], "the rest of the RX bytes should be empty");
        defmt::info!("buffer rx length > tx length - OK");
    }

    // equal rx & tx buffers
    {
        let tx_buf = [22_u8, 23, 24, 25, 26, 27];
        let mut rx_buf = [0_u8; 6];
        spi.transfer(&mut rx_buf, &tx_buf).unwrap();

        assert_eq!(rx_buf, tx_buf);
        defmt::info!("buffer rx length = tx length - OK");
    }
}
