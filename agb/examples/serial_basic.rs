//! Shows how the frame lifecycle works. How to draw objects and backgrounds,
//! and then commit that data to the screen.
#![no_std]
#![no_main]

use agb::{
    display::{
        object::{Object, SpriteVram},
        tiled::{RegularBackground, RegularBackgroundSize, TileFormat, VRAM_MANAGER},
        GraphicsFrame, Priority,
    },
    fixnum::{num, vec2, Num, Vector2D},
    include_aseprite, include_background_gfx,
    input::Button,
    input::ButtonController,
    serial::SerialBaudRate,
    serial::SerialResponse,
};

include_aseprite!(mod sprites, "examples/gfx/crab.aseprite");
include_background_gfx!(mod background, BEACH => deduplicate "examples/gfx/beach-background.aseprite");

struct Player {
    sprite: SpriteVram,
    location: Vector2D<Num<i32, 4>>,
}

impl Player {
    pub fn new(initial_location: Vector2D<Num<i32, 4>>) -> Self {
        let sprite = sprites::IDLE.sprite(0).into();

        Self {
            sprite,

            location: initial_location,
        }
    }

    pub fn update(&mut self, button_controller: &ButtonController) {
        self.location += button_controller.vector::<Num<i32, 4>>() * num!(0.5);
    }

    pub fn show(&self, frame: &mut GraphicsFrame) {
        Object::new(self.sprite.clone())
            .set_pos(self.location.floor())
            .show(frame);
    }
}

#[agb::entry]
fn main(mut gba: agb::Gba) -> ! {
    // Set up the background palettes as needed. These are produced by the include_background_gfx! macro call above.
    VRAM_MANAGER.set_background_palettes(background::PALETTES);

    // Get access to the graphics struct which is used to manage the frame lifecycle
    let mut gfx = gba.graphics.get();

    let mut player = Player::new(vec2(num!(100.), num!(100.)));
    let mut button_controller = ButtonController::new();
    let mut serial_multi_player = gba.serial.serial_multi_player(SerialBaudRate::BaudRate0);
    let mut rsp;
    let mut data = 0xDEAD;
    let mut blocking = true;

    let mut bg_tiles = RegularBackground::new(
        Priority::P0,
        RegularBackgroundSize::Background32x32,
        TileFormat::FourBpp,
    );
    bg_tiles.fill_with(&background::BEACH);

    loop {
        button_controller.update();
        serial_multi_player.sync();
        if !blocking {
            if serial_multi_player.is_active() {
                rsp = serial_multi_player.get_response();
                if rsp.sio_player_id <= 3 {
                    agb::println!(
                        "blocking rsp: {}, p1:{}, p2:{}",
                        rsp.sio_player_id,
                        rsp.sio_data[0],
                        rsp.sio_data[1]
                    );
                }
            }
        }
        if button_controller.is_pressed(Button::A) {
            serial_multi_player.activate();
        } else if button_controller.is_pressed(Button::B) {
            serial_multi_player.deactivate();
        } else if button_controller.is_pressed(Button::START) {
            data = 0xD0D0;
        } else if button_controller.is_pressed(Button::SELECT) {
            data = 0xFAFA;
        } else if button_controller.is_pressed(Button::L) {
            blocking = !blocking;
        }
        if blocking {
            if serial_multi_player.is_active() {
                rsp = serial_multi_player.transmit_data(data, blocking);
                if rsp.sio_player_id <= 3 {
                    if rsp.sio_player_id <= 3 {
                        agb::println!(
                            "not blocking rsp: {}, p1:{}, p2:{}",
                            rsp.sio_player_id,
                            rsp.sio_data[0],
                            rsp.sio_data[1]
                        );
                    }
                }
            }
        } else {
            if serial_multi_player.is_active() {
                rsp = serial_multi_player.transmit_data(data, blocking);
            }
        }
        // Update all entities in the game. In this case it is just the player, but in
        // larger games there could be more things to update.
        player.update(&button_controller);

        // Create the GraphicsFrame
        let mut frame = gfx.frame();

        // Call `.show()` on everything we want to show in this frame. If you don't call `.show()`
        // on something, it won't be visible for this frame.
        player.show(&mut frame);
        bg_tiles.show(&mut frame);

        // `.commit()` on frame will ensure that everything is drawn to the screen, and also wait
        // for the frame to finish rendering before returning control.
        frame.commit();
    }
}
