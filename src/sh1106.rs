pub const OLED_ADDR: u8 = 0x3C;

pub struct Sh1106<I2C> {
    i2c: I2C,
}

impl<I2C: embedded_hal::i2c::I2c> Sh1106<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self { i2c }
    }

    pub fn write_cmd(&mut self, cmd: u8) {
        let _ = self.i2c.write(OLED_ADDR, &[0x00, cmd]);
    }

    pub fn write_data(&mut self, data: &[u8]) {
        let mut buf = [0u8; 129];
        buf[0] = 0x40;
        let len = data.len().min(128);
        buf[1..1 + len].copy_from_slice(&data[..len]);
        let _ = self.i2c.write(OLED_ADDR, &buf[..1 + len]);
    }

    pub fn init(&mut self) {
        self.write_cmd(0xAE); // Display OFF
        self.write_cmd(0x02); // Set Lower Column Address
        self.write_cmd(0x10); // Set Higher Column Address
        self.write_cmd(0x40); // Set Display Start Line
        self.write_cmd(0xB0); // Set Page Address
        self.write_cmd(0x81);
        self.write_cmd(0xCF); // Contrast
        self.write_cmd(0xA1); // Segment Re-map
        self.write_cmd(0xC8); // COM Output Scan Direction
        self.write_cmd(0xA8);
        self.write_cmd(0x3F); // Multiplex Ratio
        self.write_cmd(0xD3);
        self.write_cmd(0x00); // Display Offset
        self.write_cmd(0xD5);
        self.write_cmd(0x80); // Display Clock Divide Ratio / Oscillator Frequency
        self.write_cmd(0xAD);
        self.write_cmd(0x8B); // DC-DC ON
        self.write_cmd(0xA4); // Entire Display OFF (RAM)
        self.write_cmd(0xA6); // Normal Display
        self.write_cmd(0xAF); // Display ON
    }

    pub fn set_pos(&mut self, page: u8, col: u8) {
        let col = col.saturating_add(2); // SH1106 132-column offset for 128px display
        self.write_cmd(0xB0 + (page & 0x07));
        self.write_cmd(col & 0x0F);
        self.write_cmd(0x10 | ((col >> 4) & 0x0F));
    }

    pub fn clear(&mut self) {
        for page in 0..8 {
            self.set_pos(page, 0);
            let zeros = [0u8; 128];
            self.write_data(&zeros);
        }
    }

    /// Clear a contiguous area of columns on a single page.
    pub fn clear_cols(&mut self, page: u8, col: u8, len: u8) {
        if len == 0 || col >= 128 {
            return;
        }
        self.set_pos(page, col);
        let max_len = (128 - col as usize).min(len as usize);
        let zeros = [0u8; 128];
        self.write_data(&zeros[..max_len]);
    }

    pub fn draw_char_6x8(&mut self, page: u8, col: u8, ch: char, font: &[[u8; 6]]) {
        let idx = (ch as u8).saturating_sub(32) as usize;
        if idx < font.len() && col + 6 <= 128 {
            self.set_pos(page, col);
            self.write_data(&font[idx]);
        }
    }

    pub fn draw_char_8x16(&mut self, page: u8, col: u8, ch: char, font: &[[u8; 16]]) {
        let idx = (ch as u8).saturating_sub(32) as usize;
        if idx < font.len() && col + 8 <= 128 {
            self.set_pos(page, col);
            self.write_data(&font[idx][0..8]);
            self.set_pos(page + 1, col);
            self.write_data(&font[idx][8..16]);
        }
    }

    pub fn draw_string_6x8(&mut self, page: u8, mut col: u8, s: &str, font: &[[u8; 6]]) {
        for ch in s.chars() {
            if col + 6 > 128 {
                break;
            }
            self.draw_char_6x8(page, col, ch, font);
            col += 6;
        }
    }

    pub fn draw_string_8x16(&mut self, page: u8, mut col: u8, s: &str, font: &[[u8; 16]]) {
        for ch in s.chars() {
            if col + 8 > 128 {
                break;
            }
            self.draw_char_8x16(page, col, ch, font);
            col += 8;
        }
    }
}
