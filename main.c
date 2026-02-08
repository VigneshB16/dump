#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "jpeg.h"

#define APLIC_BASE_ADDR        0x80100000u
#define APLIC_CLE0_OFFSET      0x001cu

#define RLE_BUFFER_BASE        0x80500000u
#define RLE_STATUS_OFFSET      0x0000u
#define RLE_COUNT_OFFSET       0x0004u
#define RLE_DATA_OFFSET        0x0008u

#define BLOCKS_PER_CHUNK       80u
#define CHUNKS_PER_FRAME       60u
#define BLOCKS_PER_FRAME       (BLOCKS_PER_CHUNK * CHUNKS_PER_FRAME)

#define JPEG_FRAME_WIDTH       640u
#define JPEG_FRAME_HEIGHT      480u

struct rle_entry {
    uint8_t run;
    int32_t value;
};

static volatile uint32_t g_rle_irq_flag;

static inline void mmio_write(uint32_t addr, uint32_t value) {
    *(volatile uint32_t *)addr = value;
}

static inline uint32_t mmio_read(uint32_t addr) {
    return *(volatile uint32_t *)addr;
}

static void clear_aplic_irq(uint32_t irq_id) {
    mmio_write(APLIC_BASE_ADDR + APLIC_CLE0_OFFSET, irq_id);
}

void __attribute__((interrupt)) external_interrupt_handler(void) {
    g_rle_irq_flag = 1u;
    clear_aplic_irq(4u);
}

static void read_rle_buffer(struct rle_entry *entries, uint32_t *entry_count) {
    uint32_t status = mmio_read(RLE_BUFFER_BASE + RLE_STATUS_OFFSET);
    uint32_t count = mmio_read(RLE_BUFFER_BASE + RLE_COUNT_OFFSET);
    uint32_t read_sel = status & 0x1u;

    uint32_t base_word = 2u;
    if (read_sel != 0u) {
        base_word = 2u;
    }

    uint32_t pair_count = count;
    uint32_t word_count = pair_count * 2u;
    (void)word_count;

    for (uint32_t i = 0; i < pair_count; i++) {
        uint32_t runlen = mmio_read(RLE_BUFFER_BASE + RLE_DATA_OFFSET + (base_word + i * 2u) * 4u);
        uint32_t value = mmio_read(RLE_BUFFER_BASE + RLE_DATA_OFFSET + (base_word + i * 2u + 1u) * 4u);
        entries[i].run = (uint8_t)(runlen & 0x3fu);
        entries[i].value = (int32_t)value;
    }

    *entry_count = pair_count;
}

static const uint8_t k_dc_bits[16] = {
    0x00, 0x01, 0x05, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t k_dc_vals[12] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b
};

static const uint8_t k_ac_bits[16] = {
    0x00, 0x02, 0x01, 0x03, 0x03, 0x02, 0x04, 0x03,
    0x05, 0x05, 0x04, 0x04, 0x00, 0x00, 0x01, 0x7d
};

static const uint8_t k_ac_vals[162] = {
    0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
    0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07,
    0x22, 0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08,
    0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16,
    0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28,
    0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
    0x4a, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
    0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79,
    0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89,
    0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
    0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7,
    0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6,
    0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
    0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2,
    0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
    0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8,
    0xf9, 0xfa
};

static void encode_rle_entries(const struct rle_entry *entries,
                               uint32_t entry_count,
                               struct jpeg_bitstream *bs,
                               const struct huffman_table *dc_table,
                               const struct huffman_table *ac_table,
                               int32_t *prev_dc,
                               uint32_t *blocks_encoded) {
    uint32_t idx = 0;

    while (idx < entry_count && *blocks_encoded < BLOCKS_PER_FRAME) {
        struct rle_entry entry = entries[idx++];
        if (entry.run != 0) {
            continue;
        }

        int32_t diff = entry.value - *prev_dc;
        *prev_dc = entry.value;

        uint8_t dc_cat = jpeg_value_category(diff);
        uint16_t dc_bits = jpeg_value_bits(diff, dc_cat);

        jpeg_bitstream_write_bits(bs, dc_table->codes[dc_cat], dc_table->sizes[dc_cat]);
        if (dc_cat > 0) {
            jpeg_bitstream_write_bits(bs, dc_bits, dc_cat);
        }

        while (idx < entry_count) {
            entry = entries[idx++];

            if (entry.run == 0 && entry.value == 0) {
                jpeg_bitstream_write_bits(bs, ac_table->codes[0x00], ac_table->sizes[0x00]);
                (*blocks_encoded)++;
                break;
            }

            if (entry.run == 15 && entry.value == 0) {
                jpeg_bitstream_write_bits(bs, ac_table->codes[0xf0], ac_table->sizes[0xf0]);
                continue;
            }

            uint8_t ac_cat = jpeg_value_category(entry.value);
            uint8_t symbol = (uint8_t)((entry.run << 4) | ac_cat);
            uint16_t ac_bits = jpeg_value_bits(entry.value, ac_cat);

            jpeg_bitstream_write_bits(bs, ac_table->codes[symbol], ac_table->sizes[symbol]);
            if (ac_cat > 0) {
                jpeg_bitstream_write_bits(bs, ac_bits, ac_cat);
            }
        }
    }
}

int main(void) {
    static uint8_t jpeg_output[JPEG_MAX_OUTPUT_BYTES];
    static struct rle_entry rle_entries[6400];

    struct huffman_table dc_table;
    struct huffman_table ac_table;
    struct jpeg_bitstream bs;

    jpeg_build_huffman_table(&dc_table, k_dc_bits, k_dc_vals, sizeof(k_dc_vals));
    jpeg_build_huffman_table(&ac_table, k_ac_bits, k_ac_vals, sizeof(k_ac_vals));

    uint32_t chunks_seen = 0;
    uint32_t blocks_encoded = 0;
    int32_t prev_dc = 0;

    g_rle_irq_flag = 0u;

    while (1) {
        if (g_rle_irq_flag == 0u) {
            continue;
        }

        g_rle_irq_flag = 0u;

        uint32_t entry_count = 0;
        read_rle_buffer(rle_entries, &entry_count);

        if (chunks_seen == 0u) {
            jpeg_bitstream_init(&bs, jpeg_output, sizeof(jpeg_output));
            jpeg_write_headers(&bs, JPEG_FRAME_WIDTH, JPEG_FRAME_HEIGHT);
            prev_dc = 0;
            blocks_encoded = 0;
        }

        encode_rle_entries(rle_entries, entry_count, &bs, &dc_table, &ac_table, &prev_dc, &blocks_encoded);
        chunks_seen++;

        if (chunks_seen >= CHUNKS_PER_FRAME) {
            jpeg_bitstream_flush(&bs);
            jpeg_write_eoi(&bs);
            chunks_seen = 0u;
        }
    }

    return 0;
}

void jpeg_bitstream_init(struct jpeg_bitstream *bs, uint8_t *buffer, size_t capacity) {
    bs->buffer = buffer;
    bs->capacity = capacity;
    bs->size = 0;
    bs->bit_buf = 0;
    bs->bit_count = 0;
}

static void jpeg_bitstream_write_byte(struct jpeg_bitstream *bs, uint8_t byte) {
    if (bs->size >= bs->capacity) {
        return;
    }
    bs->buffer[bs->size++] = byte;
    if (byte == 0xff) {
        if (bs->size >= bs->capacity) {
            return;
        }
        bs->buffer[bs->size++] = 0x00;
    }
}

void jpeg_bitstream_write_bits(struct jpeg_bitstream *bs, uint16_t bits, uint8_t count) {
    bs->bit_buf = (bs->bit_buf << count) | (bits & ((1u << count) - 1u));
    bs->bit_count += count;

    while (bs->bit_count >= 8) {
        uint8_t out = (uint8_t)(bs->bit_buf >> (bs->bit_count - 8));
        bs->bit_count -= 8;
        jpeg_bitstream_write_byte(bs, out);
    }
}

void jpeg_bitstream_flush(struct jpeg_bitstream *bs) {
    if (bs->bit_count > 0) {
        uint8_t out = (uint8_t)(bs->bit_buf << (8 - bs->bit_count));
        jpeg_bitstream_write_byte(bs, out);
        bs->bit_count = 0;
        bs->bit_buf = 0;
    }
}

void jpeg_build_huffman_table(struct huffman_table *table,
                              const uint8_t *bits,
                              const uint8_t *vals,
                              size_t vals_len) {
    uint16_t code = 0;
    size_t idx = 0;

    memset(table->codes, 0, sizeof(table->codes));
    memset(table->sizes, 0, sizeof(table->sizes));

    for (uint8_t len = 1; len <= 16; len++) {
        uint8_t count = bits[len - 1];
        for (uint8_t i = 0; i < count && idx < vals_len; i++) {
            uint8_t symbol = vals[idx++];
            table->codes[symbol] = code;
            table->sizes[symbol] = len;
            code++;
        }
        code <<= 1;
    }
}

uint8_t jpeg_value_category(int32_t value) {
    int32_t absval = value < 0 ? -value : value;
    uint8_t cat = 0;
    while (absval > 0) {
        absval >>= 1;
        cat++;
    }
    return cat;
}

uint16_t jpeg_value_bits(int32_t value, uint8_t category) {
    if (category == 0) {
        return 0;
    }
    if (value >= 0) {
        return (uint16_t)value;
    }
    int32_t mask = (1 << category) - 1;
    return (uint16_t)(value + mask);
}

static void jpeg_write_marker(struct jpeg_bitstream *bs, uint8_t marker) {
    jpeg_bitstream_write_byte(bs, 0xff);
    jpeg_bitstream_write_byte(bs, marker);
}

static void jpeg_write_word(struct jpeg_bitstream *bs, uint16_t value) {
    jpeg_bitstream_write_byte(bs, (uint8_t)(value >> 8));
    jpeg_bitstream_write_byte(bs, (uint8_t)(value & 0xff));
}

size_t jpeg_write_headers(struct jpeg_bitstream *bs, uint16_t width, uint16_t height) {
    size_t start = bs->size;

    jpeg_write_marker(bs, 0xd8);

    jpeg_write_marker(bs, 0xe0);
    jpeg_write_word(bs, 16);
    jpeg_bitstream_write_byte(bs, 'J');
    jpeg_bitstream_write_byte(bs, 'F');
    jpeg_bitstream_write_byte(bs, 'I');
    jpeg_bitstream_write_byte(bs, 'F');
    jpeg_bitstream_write_byte(bs, 0x00);
    jpeg_bitstream_write_byte(bs, 0x01);
    jpeg_bitstream_write_byte(bs, 0x01);
    jpeg_bitstream_write_byte(bs, 0x00);
    jpeg_write_word(bs, 1);
    jpeg_write_word(bs, 1);
    jpeg_bitstream_write_byte(bs, 0x00);
    jpeg_bitstream_write_byte(bs, 0x00);

    jpeg_write_marker(bs, 0xdb);
    jpeg_write_word(bs, 67);
    jpeg_bitstream_write_byte(bs, 0x00);
    for (int i = 0; i < 64; i++) {
        jpeg_bitstream_write_byte(bs, 1);
    }

    jpeg_write_marker(bs, 0xc0);
    jpeg_write_word(bs, 11);
    jpeg_bitstream_write_byte(bs, 8);
    jpeg_write_word(bs, height);
    jpeg_write_word(bs, width);
    jpeg_bitstream_write_byte(bs, 1);
    jpeg_bitstream_write_byte(bs, 1);
    jpeg_bitstream_write_byte(bs, 0x11);
    jpeg_bitstream_write_byte(bs, 0);

    jpeg_write_marker(bs, 0xc4);
    jpeg_write_word(bs, (uint16_t)(3 + 16 + sizeof(k_dc_vals)));
    jpeg_bitstream_write_byte(bs, 0x00);
    for (int i = 0; i < 16; i++) {
        jpeg_bitstream_write_byte(bs, k_dc_bits[i]);
    }
    for (size_t i = 0; i < sizeof(k_dc_vals); i++) {
        jpeg_bitstream_write_byte(bs, k_dc_vals[i]);
    }

    jpeg_write_marker(bs, 0xc4);
    jpeg_write_word(bs, (uint16_t)(3 + 16 + sizeof(k_ac_vals)));
    jpeg_bitstream_write_byte(bs, 0x10);
    for (int i = 0; i < 16; i++) {
        jpeg_bitstream_write_byte(bs, k_ac_bits[i]);
    }
    for (size_t i = 0; i < sizeof(k_ac_vals); i++) {
        jpeg_bitstream_write_byte(bs, k_ac_vals[i]);
    }

    jpeg_write_marker(bs, 0xda);
    jpeg_write_word(bs, 8);
    jpeg_bitstream_write_byte(bs, 1);
    jpeg_bitstream_write_byte(bs, 1);
    jpeg_bitstream_write_byte(bs, 0x00);
    jpeg_bitstream_write_byte(bs, 0x00);
    jpeg_bitstream_write_byte(bs, 0x3f);
    jpeg_bitstream_write_byte(bs, 0x00);

    return bs->size - start;
}

void jpeg_write_eoi(struct jpeg_bitstream *bs) {
    jpeg_write_marker(bs, 0xd9);
}
