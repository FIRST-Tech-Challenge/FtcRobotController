#ifndef _SBR_H_
#define _SBR_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stdbool.h"

#define SBR_MAX_NUM_SECTIONS (17)
#define SBR_SECTION_NAME_MAX_SIZE (16)
#define SBR_IDENTIFIER_SIZE (2)

#define SBR_SECTION_FLAG_BOOTABLE (1 << 0)
#define SBR_SECTION_FLAG_IGNORE_CHECKSUM (1 << 1)
#define SBR_SECTION_FLAG_COMPRESSION_MASK (0x7 << 2)
typedef enum {
    SBR_NO_COMPRESSION = 0U << 2,
    SBR_COMPRESSION_ZLIB = 1U << 2,
    SBR_COMPRESSION_GZ = 2U << 2,
    SBR_COMPRESSION_XZ = 3U << 2,
} SBR_COMPRESSION;

static const uint8_t SBR_IDENTIFIER[SBR_IDENTIFIER_SIZE] = {'B', 'R'};

// SBR_SECTION
typedef struct {
    char name[SBR_SECTION_NAME_MAX_SIZE];
    uint32_t size;
    uint32_t offset;
    uint32_t checksum;
    uint8_t type;
    uint8_t flags;
} SBR_SECTION;


// RAW_SBR_SECTION
typedef struct {
    uint8_t bytes[  sizeof( ((SBR_SECTION*)0)->name ) +\
                    sizeof( ((SBR_SECTION*)0)->size ) +\
                    sizeof( ((SBR_SECTION*)0)->offset ) +\
                    sizeof( ((SBR_SECTION*)0)->checksum ) +\
                    sizeof( ((SBR_SECTION*)0)->type ) +\
                    sizeof( ((SBR_SECTION*)0)->flags ) ];
} SBR_SECTION_RAW;


// SBR structure
typedef struct {
    uint8_t identifier[SBR_IDENTIFIER_SIZE];
    SBR_SECTION sections[SBR_MAX_NUM_SECTIONS];
} SBR;

#define SBR_RAW_SIZE (sizeof(SBR_SECTION_RAW) * SBR_MAX_NUM_SECTIONS + SBR_IDENTIFIER_SIZE)

int sbr_parse(const void* buffer, uint32_t size, SBR* sbr);
int sbr_serialize(const SBR* sbr, void* buffer, uint32_t max_size);

uint32_t sbr_initial_checksum();
uint32_t sbr_compute_checksum_prev(const void* buffer, uint32_t size, uint32_t prev_checksum);
uint32_t sbr_compute_checksum(const void* buffer, uint32_t size);

void sbr_section_set_name(SBR_SECTION* sbr_section, const char* name);
void sbr_section_set_size(SBR_SECTION* sbr_section, uint32_t size);
void sbr_section_set_offset(SBR_SECTION* sbr_section, uint32_t offset);
void sbr_section_set_checksum(SBR_SECTION* sbr_section, uint32_t checksum);
void sbr_section_set_type(SBR_SECTION* sbr_section, uint8_t type);
void sbr_section_set_bootable(SBR_SECTION* sbr_section, bool bootable);
void sbr_section_set_ignore_checksum(SBR_SECTION* sbr_section, bool ignore_checksum);
void sbr_section_set_compression(SBR_SECTION* sbr_section, SBR_COMPRESSION compression);

bool sbr_section_get_bootable(const SBR_SECTION* sbr_section);
bool sbr_section_get_ignore_checksum(const SBR_SECTION* sbr_section);
bool sbr_section_is_valid(const SBR_SECTION* sbr_section);
SBR_COMPRESSION sbr_section_get_compression(const SBR_SECTION* sbr_section);

#ifdef __cplusplus
}
#endif

#endif // _SBR_H_
