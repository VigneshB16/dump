#ifndef JPEG_H
#define JPEG_H

#include <stdint.h>
#include <stddef.h>

#define JPEG_MAX_OUTPUT_BYTES (2u * 1024u * 1024u)

struct jpeg_bitstream {
    uint8_t *buffer;
    size_t capacity;
    size_t size;
    uint32_t bit_buf;
    uint8_t bit_count;
};

struct huffman_table {
    uint16_t codes[256];
    uint8_t sizes[256];
};

void jpeg_bitstream_init(struct jpeg_bitstream *bs, uint8_t *buffer, size_t capacity);
void jpeg_bitstream_write_bits(struct jpeg_bitstream *bs, uint16_t bits, uint8_t count);
void jpeg_bitstream_flush(struct jpeg_bitstream *bs);

void jpeg_build_huffman_table(struct huffman_table *table,
                              const uint8_t *bits,
                              const uint8_t *vals,
                              size_t vals_len);

size_t jpeg_write_headers(struct jpeg_bitstream *bs, uint16_t width, uint16_t height);
void jpeg_write_eoi(struct jpeg_bitstream *bs);

uint8_t jpeg_value_category(int32_t value);
uint16_t jpeg_value_bits(int32_t value, uint8_t category);

#endif
