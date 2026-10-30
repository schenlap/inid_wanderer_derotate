/*******************************************************************************
 Field derotation helper for alt-az mounts (header-only, no INDI dependency).

 Intended to be used inside an INDI rotator driver (e.g. WandererRotator Mini):
 the driver snoops the mount (EQUATORIAL_EOD_COORD, GEOGRAPHIC_COORD), feeds the
 values into Derotator::step() every second or so and moves the rotator to the
 returned absolute angle.

 Field rotation rate of an alt-az mount:
     dTheta/dt = w_sid * cos(lat) * cos(az) / cos(alt)     (az from north)

 License: GPL-2.0-or-later (same as INDI).
*******************************************************************************/
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>

namespace FieldDerotation
{

constexpr double SIDEREAL_RATE_DEG_S = 360.0 / 86164.0905;   // ~0.0041781 deg/s
constexpr double PI_ = 3.14159265358979323846;

inline double rad(double d) { return d * PI_ / 180.0; }
inline double deg(double r) { return r * 180.0 / PI_; }

inline double wrap360(double a)
{
    a = std::fmod(a, 360.0);
    return a < 0 ? a + 360.0 : a;
}

inline double julianDateNow()
{
    using namespace std::chrono;
    double unix_s = duration_cast<duration<double>>(system_clock::now().time_since_epoch()).count();
    return unix_s / 86400.0 + 2440587.5;
}

// RA in hours (JNow), Dec/lat/lon in degrees (lon east positive). az from north via east.
inline void azAltFromRaDec(double raHours, double decDeg, double latDeg, double lonDeg,
                           double jd, double &azDeg, double &altDeg)
{
    double d    = jd - 2451545.0;
    double gmst = wrap360(280.46061837 + 360.98564736629 * d);
    double lst  = wrap360(gmst + lonDeg);
    double ha   = rad(lst - raHours * 15.0);
    double dec  = rad(decDeg), lat = rad(latDeg);

    double sinAlt = std::sin(dec) * std::sin(lat) + std::cos(dec) * std::cos(lat) * std::cos(ha);
    altDeg = deg(std::asin(std::max(-1.0, std::min(1.0, sinAlt))));
    azDeg  = wrap360(deg(std::atan2(-std::cos(dec) * std::sin(ha),
                                    std::sin(dec) * std::cos(lat) - std::cos(dec) * std::sin(lat) * std::cos(ha))));
}

// Field rotation rate in deg/s (positive for azimuths near north).
inline double fieldRotationRate(double latDeg, double azDeg, double altDeg)
{
    double cosAlt = std::max(std::cos(rad(altDeg)), 1e-6);
    return SIDEREAL_RATE_DEG_S * std::cos(rad(latDeg)) * std::cos(rad(azDeg)) / cosAlt;
}

class Derotator
{
public:
    void setTrimPercent(double p) { m_factor = 1.0 + p / 100.0; }   // e.g. +0.3 = 0.3 % faster
    void setMaxRate(double degPerSec) { m_maxRate = degPerSec; }    // limit near zenith
    void setMinStep(double deg_) { m_minStep = deg_; }              // smallest commanded step
    void setReverse(bool r) { m_sign = r ? 1.0 : -1.0; }            // rotate with/against sign convention

    bool active() const { return m_active; }
    double lastRate() const { return m_lastRate; }                  // deg/s incl. trim
    bool rateLimited() const { return m_limited; }

    void start(double currentAngle, double nowSeconds)
    {
        m_ideal = m_commanded = wrap360(currentAngle);
        m_pending = 0.0;
        m_lastT = nowSeconds;
        m_active = true;
    }
    void stop() { m_active = false; }

    // Call periodically. Returns true if the rotator should be moved to 'target'.
    // Call confirmSent() afterwards when the move was actually issued.
    bool step(double az, double alt, double lat, double nowSeconds, double &target)
    {
        if (!m_active)
            return false;
        double rate = fieldRotationRate(lat, az, alt);
        m_limited   = std::fabs(rate) > m_maxRate;
        rate        = std::max(-m_maxRate, std::min(m_maxRate, rate)) * m_factor;
        m_lastRate  = rate;

        double dt = nowSeconds - m_lastT;
        m_lastT   = nowSeconds;
        m_ideal   = wrap360(m_ideal + m_sign * rate * dt);

        double diff = std::fmod(m_ideal - m_commanded + 540.0, 360.0) - 180.0;
        if (std::fabs(diff) < m_minStep)
            return false;
        target = m_ideal;
        return true;
    }
    void confirmSent(double target) { m_commanded = wrap360(target); }

    // --- Variante mit relativen Schritten (für Rotatoren mit Relativ-Kommando) ---
    // Integriert die Drehrate; liefert true, sobald die aufgelaufene Korrektur >= minStep ist.
    // 'delta' ist die vorzeichenbehaftete Korrektur in Grad. Nach erfolgreichem Senden
    // consumed(delta) aufrufen, sonst bleibt der Betrag erhalten und wird später nachgeholt.
    bool stepRelative(double az, double alt, double lat, double nowSeconds, double &delta)
    {
        if (!m_active)
            return false;
        double rate = fieldRotationRate(lat, az, alt);
        m_limited   = std::fabs(rate) > m_maxRate;
        rate        = std::max(-m_maxRate, std::min(m_maxRate, rate)) * m_factor;
        m_lastRate  = rate;

        m_pending += m_sign * rate * (nowSeconds - m_lastT);
        m_lastT = nowSeconds;
        if (std::fabs(m_pending) < m_minStep)
            return false;
        delta = m_pending;
        return true;
    }
    void consumed(double delta) { m_pending -= delta; }
    double pending() const { return m_pending; }

private:
    bool   m_active    = false;
    bool   m_limited   = false;
    double m_factor    = 1.0;
    double m_maxRate   = 0.05;
    double m_minStep   = 0.01;
    double m_sign      = -1.0;
    double m_ideal     = 0.0;
    double m_commanded = 0.0;
    double m_lastT     = 0.0;
    double m_lastRate  = 0.0;
    double m_pending   = 0.0;
};

} // namespace FieldDerotation
