/*
    Copyright (c) 2017-2021 Ultimaker B.V. All rights reserved.

    Marlin is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Marlin is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Marlin.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef GAIN_COMPENSATOR
#define GAIN_COMPENSATOR

/** @brief The gain compensator allows for a gain on a measured value, using a factor and offset
 */
class GainCompensator
{
public:
    /** @brief Default constructor with settings for no compensation
     *  @param factor The factor used to multiply the given value
     *  @param offset The offset to be added (or subtracted) after applying the factor
     */
    GainCompensator(const float factor=1.0, const float offset=0.0)
    {
        this->factor = factor;
        this->offset = offset;
    };

    /** @brief Sets the factor
     *  @param factor The factor used to multiply the given value
     */
    inline void setFactor(const float factor)
    {
        this->factor = factor;
    }

    /** @brief Gets the factor
     *  @return Returns the current factor
     */
    inline float getFactor() const
    {
        return this->factor;
    }

    /** @brief Sets the offset
     *  @param offset The offset to be added (or subtracted) after applying the factor
     */
    inline void setOffset(const float offset)
    {
        this->offset = offset;
    }

    /** @brief Gets the offset
     *  @return Returns the current offset
     */
    inline float getOffset() const
    {
        return this->offset;
    }

    /** @brief Compensates the given value using a gain by multiplying with the factor and adding the offset
     *  @param value The measured value to which the gain should be applied
     *  @return Returns the compensated value
     */
    inline float compensate(const float value) const
    {
        return (value * this->factor) + this->offset;
    }

private:
    /** @brief The factor to apply before offset is added */
    float factor;
    /** @brief The offset to be added after factor is used */
    float offset;
};

#endif//GAIN_COMPENSATOR