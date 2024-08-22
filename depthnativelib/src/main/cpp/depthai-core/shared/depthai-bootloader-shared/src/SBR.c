#include "depthai-bootloader-shared/SBR.h"

#include "assert.h"
#include "stddef.h"
#include "string.h"

static uint32_t read_uint32_t(const void* buffer) {
    if(buffer == NULL) return 0;
    uint8_t* p = (uint8_t*)buffer;
    uint32_t val = 0;
    val = (p[0]) + (p[1] << 8) + (p[2] << 16) + (p[3] << 24);
    return val;
}

static void write_uint32_t(void* buffer, uint32_t val) {
    if(buffer == NULL) return;
    uint8_t* p = (uint8_t*)buffer;
    for(int i = 0; i < 4; i++) {
        *p++ = val & 0xFF;
        val = val >> 8;
    }
}

static int read_section(const void* buffer, uint32_t size, SBR_SECTION* section) {
    if(section == NULL) return -1;
    if(size < sizeof(SBR_SECTION_RAW)) return -1;

    const uint8_t* p = (const uint8_t*)buffer;

    // Read name
    memcpy(section->name, p, sizeof(section->name));
    p += sizeof(section->name);

    // Read size
    section->size = read_uint32_t(p);
    p += sizeof(section->size);

    // Read offset
    section->offset = read_uint32_t(p);
    p += sizeof(section->offset);

    // Read checksum
    section->checksum = read_uint32_t(p);
    p += sizeof(section->checksum);

    // Read type
    section->type = read_uint32_t(p);
    p += sizeof(section->type);

    // Read flags
    section->flags = read_uint32_t(p);
    p += sizeof(section->flags);

    return 0;
}

int sbr_parse(const void* buffer, uint32_t size, SBR* sbr) {
    if(buffer == NULL) return -1;
    if(sbr == NULL) return -1;
    if(size < SBR_RAW_SIZE) return -1;

    // pointer to be moved to current data being parsed
    const uint8_t* p = (uint8_t*)buffer;

    // copy identifier
    memcpy(sbr->identifier, p, SBR_IDENTIFIER_SIZE);
    p += sizeof(sbr->identifier);

    // check identifier
    if(memcmp(sbr->identifier, SBR_IDENTIFIER, SBR_IDENTIFIER_SIZE) != 0) {
        return -1;
    }

    for(unsigned int i = 0; i < SBR_MAX_NUM_SECTIONS; i++) {
        if(read_section(p, sizeof(SBR_SECTION_RAW), &sbr->sections[i])) {
            return -1;
        }
        p += sizeof(SBR_SECTION_RAW);
    }

    return 0;
}

int sbr_serialize(const SBR* sbr, void* buffer, uint32_t max_size) {
    if(buffer == NULL) return -1;
    if(sbr == NULL) return -1;
    if(max_size < SBR_RAW_SIZE) return -1;

    // pointer to be moved to location of serialized data
    uint8_t* p = (uint8_t*)buffer;

    // write out identifier
    memcpy(p, SBR_IDENTIFIER, sizeof(SBR_IDENTIFIER));
    p += sizeof(SBR_IDENTIFIER);

    // write out sections
    for(unsigned int i = 0; i < SBR_MAX_NUM_SECTIONS; i++) {
        // write name
        memcpy(p, sbr->sections[i].name, sizeof(sbr->sections[i].name));
        p += sizeof(sbr->sections[i].name);

        // write size
        write_uint32_t(p, sbr->sections[i].size);
        p += sizeof(sbr->sections[i].size);

        // write offset
        write_uint32_t(p, sbr->sections[i].offset);
        p += sizeof(sbr->sections[i].offset);

        // write checksum
        write_uint32_t(p, sbr->sections[i].checksum);
        p += sizeof(sbr->sections[i].checksum);

        // write type
        *p++ = sbr->sections[i].type;

        // write flags
        *p++ = sbr->sections[i].flags;
    }

    // write out identifier
    memcpy(p, SBR_IDENTIFIER, sizeof(SBR_IDENTIFIER));
    p += sizeof(SBR_IDENTIFIER);

    return 0;
}

bool sbr_section_get_bootable(const SBR_SECTION* sbr_section) {
    if(sbr_section == NULL) return false;
    if(sbr_section->flags & SBR_SECTION_FLAG_BOOTABLE) return true;
    return false;
}

bool sbr_section_get_ignore_checksum(const SBR_SECTION* sbr_section) {
    if(sbr_section == NULL) return false;
    if(sbr_section->flags & SBR_SECTION_FLAG_IGNORE_CHECKSUM) return true;
    return false;
}

SBR_COMPRESSION sbr_section_get_compression(const SBR_SECTION* sbr_section) {
    if(sbr_section == NULL) return SBR_NO_COMPRESSION;
    return sbr_section->flags & SBR_SECTION_FLAG_COMPRESSION_MASK;
}

bool sbr_section_is_valid(const SBR_SECTION* sbr_section) {
    // Valid SBR section must have a name set, as well as a non zero size
    if(sbr_section->name[0] == 0 || ((uint8_t)sbr_section->name[0]) == 0xFF) {
        return false;
    }
    return true;
}

uint32_t sbr_initial_checksum() {
    return 5381;
}

uint32_t sbr_compute_checksum_prev(const void* buffer, uint32_t size, uint32_t prev_checksum) {
    uint32_t checksum = prev_checksum;
    uint8_t* p = (uint8_t*)buffer;

    for(unsigned int i = 0; i < size; i++) {
        checksum = ((checksum << 5) + checksum) + p[i]; /* hash * 33 + p[i] */
    }

    return checksum;
}

uint32_t sbr_compute_checksum(const void* buffer, uint32_t size) {
    return sbr_compute_checksum_prev(buffer, size, sbr_initial_checksum());
}

void sbr_section_set_name(SBR_SECTION* sbr_section, const char* name) {
    assert(sbr_section != NULL);
    strncpy(sbr_section->name, name, SBR_SECTION_NAME_MAX_SIZE);
}

void sbr_section_set_size(SBR_SECTION* sbr_section, uint32_t size) {
    assert(sbr_section != NULL);
    sbr_section->size = size;
}

void sbr_section_set_offset(SBR_SECTION* sbr_section, uint32_t offset) {
    assert(sbr_section != NULL);
    sbr_section->offset = offset;
}

void sbr_section_set_checksum(SBR_SECTION* sbr_section, uint32_t checksum) {
    assert(sbr_section != NULL);
    sbr_section->checksum = checksum;
}

void sbr_section_set_type(SBR_SECTION* sbr_section, uint8_t type) {
    assert(sbr_section != NULL);
    sbr_section->type = type;
}

void sbr_section_set_bootable(SBR_SECTION* sbr_section, bool bootable) {
    assert(sbr_section != NULL);
    if(bootable) {
        sbr_section->flags |= SBR_SECTION_FLAG_BOOTABLE;
    } else {
        sbr_section->flags &= ~SBR_SECTION_FLAG_BOOTABLE;
    }
}

void sbr_section_set_ignore_checksum(SBR_SECTION* sbr_section, bool ignore_checksum) {
    assert(sbr_section != NULL);
    if(ignore_checksum) {
        sbr_section->flags |= SBR_SECTION_FLAG_IGNORE_CHECKSUM;
    } else {
        sbr_section->flags &= ~SBR_SECTION_FLAG_IGNORE_CHECKSUM;
    }
}

void sbr_section_set_compression(SBR_SECTION* sbr_section, SBR_COMPRESSION compression) {
    assert(sbr_section != NULL);

    // Clear previous compression
    sbr_section->flags &= ~SBR_SECTION_FLAG_COMPRESSION_MASK;

    // Set new compression
    sbr_section->flags |= compression;
}
