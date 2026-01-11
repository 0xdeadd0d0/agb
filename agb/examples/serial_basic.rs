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
    serial::SerialMultiPlayer,
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

    pub fn serial_send(&self, serial: &mut SerialMultiPlayer) {
        let x_raw = Num::to_raw(self.location.x);
        let y_raw = Num::to_raw(self.location.y);
        serial.send_data(0xd0d0);
        let data = (x_raw as u32 & 0xFFFF) as u16;
        serial.send_data(if data == 0 { 0xdead } else { data });
        serial.send_data(0xdada);
        let data = (y_raw as u32 & 0xFFFF) as u16;
        serial.send_data(if data == 0 { 0xdead } else { data });
    }

    pub fn serial_rcv(&mut self, serial: &mut SerialMultiPlayer, player_id: usize) {
        let mut x_raw = Num::to_raw(self.location.x);
        let mut y_raw = Num::to_raw(self.location.y);
        let mut data0 = serial.get_player_rsp(player_id).unwrap_or(0x0);
        let mut data1 = serial.get_player_rsp(player_id).unwrap_or(0x0);

        while data0 != 0 {
            if (data0 == 0xd0d0)
                && (data1 != 0xd0d0)
                && (data1 != 0xdada)
                && (data1 != 0xdead)
                && (data1 != 0)
            {
                x_raw = ((x_raw as u32 & 0xFFFF0000 as u32) | (data1 as u32)) as i32;
            } else if (data0 == 0xdada)
                && (data1 != 0xd0d0)
                && (data1 != 0xdada)
                && (data1 != 0xdead)
                && (data1 != 0)
            {
                y_raw = ((y_raw as u32 & 0xFFFF0000 as u32) | (data1 as u32)) as i32;
            }
            data0 = serial.get_player_rsp(player_id).unwrap_or(0x0);
            data1 = serial.get_player_rsp(player_id).unwrap_or(0x0);
        }
        self.location.x = Num::from_raw(x_raw);
        self.location.y = Num::from_raw(y_raw);
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

    let mut players = [
        Player::new(vec2(num!(25.), num!(25.))),
        Player::new(vec2(num!(50.), num!(50.))),
        Player::new(vec2(num!(75.), num!(75.))),
        Player::new(vec2(num!(100.), num!(100.))),
    ];
    let mut button_controller = ButtonController::new();
    let mut serial_multi_player = gba.serial.serial_multi_player(SerialBaudRate::BaudRate3);

    let mut bg_tiles = RegularBackground::new(
        Priority::P0,
        RegularBackgroundSize::Background32x32,
        TileFormat::FourBpp,
    );
    bg_tiles.fill_with(&background::BEACH);

    loop {
        button_controller.update();
        serial_multi_player.sync();
        let player_id = serial_multi_player.get_player_id();
        let players_online = serial_multi_player.get_nb_players_online();

        if serial_multi_player.is_active() {
            for i in 0..players_online {
                if i != player_id {
                    players[i].serial_rcv(&mut serial_multi_player, i);
                }
            }
            if player_id < 4 {
                players[player_id].serial_send(&mut serial_multi_player);
                // Update all entities in the game. In this case it is just the player, but in
                // larger games there could be more things to update.
                players[player_id].update(&button_controller);
            }
        }
        if button_controller.is_pressed(Button::A) {
            serial_multi_player.activate();
        } else if button_controller.is_pressed(Button::B) {
            serial_multi_player.deactivate();
        }

        // Create the GraphicsFrame
        let mut frame = gfx.frame();

        if serial_multi_player.is_active() {
            for i in 0..players_online {
                // Call `.show()` on everything we want to show in this frame. If you don't call `.show()`
                // on something, it won't be visible for this frame.
                players[i].show(&mut frame);
            }
        }
        bg_tiles.show(&mut frame);

        // `.commit()` on frame will ensure that everything is drawn to the screen, and also wait
        // for the frame to finish rendering before returning control.
        frame.commit();
    }
}
