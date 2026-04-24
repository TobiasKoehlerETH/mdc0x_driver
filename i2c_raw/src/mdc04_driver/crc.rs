pub fn crc8_i2c(data: &[u8]) -> u8 {
    let mut crc: u8 = 0xFF;
    let mut i = 0usize;
    while i < data.len() {
        crc ^= data[i];
        let mut bit = 0u8;
        while bit < 8 {
            if (crc & 0x80) != 0 {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
            bit += 1;
        }
        i += 1;
    }
    crc
}
