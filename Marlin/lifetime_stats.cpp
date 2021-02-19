#include "lifetime_stats.h"
#include <avr/eeprom.h>
#include "Marlin.h"
#include "fan_driver.h"
#include "temperature.h"
#include "crc8.h"


// The EEPROM has a guaranteed 100.000 erase cycles. By writing once an hour we get about 11 years of continuous service
// which should be enough to last a lifetime.
#define MILLIS_MINUTE (1000L * 60L)
#define MILLIS_HOUR (MILLIS_MINUTE * 60L)
#define LIFETIME_EEPROM_OFFSET 0x100    // First address for storing the lifetime data
#define LIFETIME_VERSION_NR 1   // Current version number for the data structure. Useful for version upgrades
#define REPLICATION 3           // Number of data copies for recovery and reducing wear


static uint32_t minute_counter_millis;
static uint32_t hour_save_millis;
static uint32_t last_e_pos;

static uint8_t next_replication_index = 0; // Use this index for the next time we write to eeprom

struct lifetime_storage lifetime;


// Local functions
static void update_minute_counters();
static bool is_printing();
static void lifetime_stats_load();
static void lifetime_stats_save();
static void set_lifetime_to_default_values();
static bool verify_checksum(struct lifetime_storage *target_lifetime);
static void set_checksum(struct lifetime_storage *target_lifetime);


void lifetime_stats_init()
{
    const uint32_t now = millis();
    hour_save_millis = now;
    minute_counter_millis = now ;
    last_e_pos = current_position[E_AXIS];

    lifetime_stats_load();
}

void lifetime_stats_update()
{
    const uint32_t now = millis();

    // Every minute, increase the active minute counters
    if (now - minute_counter_millis > MILLIS_MINUTE)    // Unsigned integer subtraction, no need to check for overflow
    {
        minute_counter_millis += MILLIS_MINUTE;
        update_minute_counters();
    }

    // Every hour, save the data to EEPROM
    if (now - hour_save_millis > MILLIS_HOUR)   // Unsigned integer subtraction, no need to check for overflow
    {
        hour_save_millis += MILLIS_HOUR;
        lifetime_stats_save();
    }
}

void lifetime_stats_force_write_to_eeprom()
{
    const uint32_t now = millis();
    hour_save_millis = now + MILLIS_HOUR;
    
    lifetime_stats_save();
}

// Internal functions

static void update_minute_counters()
{
    lifetime.on_minutes++;

    if (topcap_fan_is_on)
    {
        lifetime.topcap_fan_minutes++;
    }

    if (is_printing())
    {
        lifetime.print_minutes++;

        // The E position tracking here has some flaws like not taking into account forced E92 position
        // updates, but since we only track the positive moves we are not too far off. Perhaps we loose a few
        // minutes of data, but on the big picture that's peanuts.
        int32_t diff = int32_t(current_position[E_AXIS]) - last_e_pos;
        if (diff > 0)
        {
            lifetime.print_millimeters += diff;
        }
        last_e_pos = current_position[E_AXIS];
    }
}

// We assume to be printing when at least one extruder >= 60C
static bool is_printing()
{
    for (uint8_t e = 0; e < EXTRUDERS; e++)
    {
        if (current_temperature[e] >= 60)
        {
            return true;
        }
    }

    return false;
}

// Return True when the checksum in the storage container is correct
static bool verify_checksum(struct lifetime_storage *target_lifetime)
{
    Crc8 checksum;

    uint8_t crc_backup = target_lifetime->crc;
    target_lifetime->crc = 0; // Always have the crc field at 0 for the crc calculation

    checksum.reset();
    checksum.update(target_lifetime, sizeof(*target_lifetime));

    target_lifetime->crc = crc_backup;

    return checksum.getCrc() == target_lifetime->crc;
}

// Sets the checksum field in the storage container
static void set_checksum(struct lifetime_storage *target_lifetime)
{
    Crc8 checksum;

    target_lifetime->crc = 0; // Always have the crc field at 0 for the crc calculation

    checksum.reset();
    checksum.update(target_lifetime, sizeof(*target_lifetime));

    target_lifetime->crc = checksum.getCrc();
}

static void lifetime_stats_save()
{
    // To prevent writing to the lifetime struct while we are writing to the eeprom
    // Make a local copies, then set the crc and write to eeprom
    struct lifetime_storage local_lifetime = lifetime;

    set_checksum(&local_lifetime);

    // Calculated the actual address in eeprom where we want to store the lifetime
    uint16_t target_address = LIFETIME_EEPROM_OFFSET + next_replication_index * sizeof(lifetime);

    eeprom_write_block(&local_lifetime, (void *)target_address, sizeof(local_lifetime));

    next_replication_index = (next_replication_index + 1) % REPLICATION;
}

static void lifetime_stats_load()
{
    set_lifetime_to_default_values();

    uint32_t lowest_on_minutes = (uint32_t)-1;

    for (uint8_t i=0; i < REPLICATION; i++)
    {
        uint16_t address = LIFETIME_EEPROM_OFFSET + i * sizeof(lifetime);
        
        struct lifetime_storage local_lifetime;

        eeprom_read_block(&local_lifetime, (const void *)address, sizeof(local_lifetime));

        // Was this a virgin installation?
        if (local_lifetime.version == 0xFF)
        {
            continue;   // Read next block
        }

        // Was this a bad block?
        if (!verify_checksum(&local_lifetime))
        {   
            continue;   // Read next block
        }

        // We have a valid block, now if this block has the highest amount of `on_minutes` consider it the current best and store it for later use
        if (local_lifetime.on_minutes > lifetime.on_minutes)
        {
            lifetime = local_lifetime;
        }
        
        // If the valid block has the lowest amount of `on_minutes`, store its index to overwrite it first
        if (local_lifetime.on_minutes < lowest_on_minutes)
        {
            lowest_on_minutes = local_lifetime.on_minutes;
            next_replication_index = i;
        }
    }

}

static void set_lifetime_to_default_values()
{
    memset(&lifetime, 0, sizeof(lifetime));
    lifetime.version = LIFETIME_VERSION_NR;
}
