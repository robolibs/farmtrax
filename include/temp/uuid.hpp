#pragma once

#include <iomanip>
#include <random>
#include <sstream>
#include <string>

namespace temp {

    /**
     * @brief Generate a UUID v4 (random) string
     *
     * Generates a random UUID in the format: xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx
     * where x is any hex digit and y is one of 8, 9, a, or b.
     *
     * @return UUID string
     */
    inline std::string generate_uuid() {
        static std::random_device rd;
        static std::mt19937_64 gen(rd());
        static std::uniform_int_distribution<uint64_t> dis;

        uint64_t ab = dis(gen);
        uint64_t cd = dis(gen);

        // Set version to 4 (random UUID)
        ab = (ab & 0xFFFFFFFFFFFF0FFFULL) | 0x0000000000004000ULL;
        // Set variant to RFC 4122
        cd = (cd & 0x3FFFFFFFFFFFFFFFULL) | 0x8000000000000000ULL;

        std::stringstream ss;
        ss << std::hex << std::setfill('0');

        ss << std::setw(8) << ((ab >> 32) & 0xFFFFFFFF) << "-";
        ss << std::setw(4) << ((ab >> 16) & 0xFFFF) << "-";
        ss << std::setw(4) << (ab & 0xFFFF) << "-";
        ss << std::setw(4) << ((cd >> 48) & 0xFFFF) << "-";
        ss << std::setw(12) << (cd & 0xFFFFFFFFFFFFULL);

        return ss.str();
    }

} // namespace temp
