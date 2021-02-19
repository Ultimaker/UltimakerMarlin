#ifndef LIFETIME_STATS_H
#define LIFETIME_STATS_H


#include <stdint.h>
#include "crc8.h"

/*
This class implements a non-volatile storage facility in the internal EEPROM memory of the ATmega256 processor.

Physical limitations of the EEPROM that we have to take into account:
- 4 kbytes storage
- Each byte is individually programmable by the processor (in contrast to external programmers which access the
  EEPROM as 512 pages of 8 bytes each).
- A typical endurance of 100.000 writes and unlimited reads
- Writing a single byte takes 3.3ms


The intention is to keep this class robust but simple. Storage errors will be detected and recovered but
simplifications were made by:
- There is no wear leveling, this can be added later. Assuming 1 write per hour we can update data for 11 years
  continuous running time, that's 20+ years life time. Note that the data recovery feature will multiply the life time
  as a side effect.
- Writing and reading the EEPROM are blocking functions, especially the writing is slow and can take 3ms per
  modified byte.
  This can be improved upon in multiple ways, but for now we keep it simple. Tests have shown that even an
  occasional 100ms delay is invisible in most printed objects.
- We detect corrupted data and will recover from this by falling back to a previous record, i.e. there will be
  some data loss but this has the advantage that we only write one record at a time instead of multiple records.
- There is no tracking of bad reads, i.e. we don't mark pages as damaged. This was chosen since it's not easy
  to see the difference between a corrupted record from a power fail over a physical damaged EEPROM byte.


Basic principle of operation:
- Start by calling the init() function. This will create a new data structure when no previous data exists in the EEPROM.
- You can update storage data as often as you want. The catch is that the data will only be written to EEPROM once
  every hour, this to avoid wearing out the EEPROM cells. In case of a reset or power loss you loose maximum 1 hour of
  data.
*/


// An encapsulation object for storing data in EEPROM
// Writing a single modified byte takes considerable time of 3ms., so keep this object small.
struct lifetime_storage
{
    uint8_t crc;                // CRC8 checksum
    uint8_t version;            // Version number for this data struct so we can update, make sure this is always in this location

    uint32_t on_minutes;
    uint32_t print_minutes;
    uint32_t print_millimeters; // 750g spool of PLA at 2.85mm is about 90m long. In a uint32 with mm accuracy we can then track 6 spools a day for 20 years.
    uint32_t topcap_fan_minutes;
};
extern struct lifetime_storage lifetime;

// Initialize the lifetime statistics module.
// When no previous data exists in the EEPROM this routine will create default values.
void lifetime_stats_init();

// Do regular update things.
// Call at slow interval pace, once per minute is okay.
void lifetime_stats_update();

// When a print is done, force the lifetime stats to be written to eeprom
void lifetime_stats_force_write_to_eeprom();

#endif // LIFETIME_STATS_H
