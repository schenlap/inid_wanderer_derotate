/*******************************************************************************
  Copyright(c) 2025 Frank Wang & Jérémie Klein. All rights reserved.

  WandererRotator Lite V1

  This program is free software; you can redistribute it and/or modify it
  under the terms of the GNU General Public License as published by the Free
  Software Foundation; either version 2 of the License, or (at your option)
  any later version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU Library General Public License
  along with this library; see the file COPYING.LIB.  If not, write to
  the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
  Boston, MA 02110-1301, USA.

  The full GNU General Public License is included in this distribution in the
  file called LICENSE.
*******************************************************************************/

#include "wanderer_rotator_base.h"
#include "indicom.h"
#include "connectionplugins/connectionserial.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <iostream>

// Signed difference of two angles mapped to (-180, 180]
static inline double wrap180Deg(double a)
{
    return FieldDerotation::wrap360(a + 180.0) - 180.0;
}


WandererRotatorBase::WandererRotatorBase()
{
    setVersion(1, 0);

}

bool WandererRotatorBase::initProperties()
{

    INDI::Rotator::initProperties();

    SetCapability(ROTATOR_CAN_REVERSE | ROTATOR_CAN_ABORT | ROTATOR_CAN_HOME);

    addAuxControls();
    // Calibrate
    SetZeroSP[0].fill("Set_Zero", "Mechanical Zero", ISS_OFF);
    SetZeroSP.fill(getDeviceName(), "Set_Zero", "Set Current As", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 60, IPS_IDLE);

    // BACKLASH
    BacklashNP[BACKLASH].fill( "BACKLASH", "Degree", "%.2f", 0, 3, 0.1, 0);
    BacklashNP.fill(getDeviceName(), "BACKLASH", "Backlash", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);

    // ACCURACY
    AccuracyNP[0].fill("ACCURACY", "Degree (0=off)", "%.2f", 0, 10, 0.1, 0);
    AccuracyNP.fill(getDeviceName(), "ACCURACY", "Accuracy", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);
    AccuracyNP.load();

    // ---------------- Field derotation ----------------
    static const char *DEROTATE_TAB = "Derotation";

    DerotateSP[0].fill("DEROTATE_ON", "Enable", ISS_OFF);
    DerotateSP[1].fill("DEROTATE_OFF", "Disable", ISS_ON);
    DerotateSP.fill(getDeviceName(), "FIELD_DEROTATION", "Field Derotation", DEROTATE_TAB, IP_RW, ISR_1OFMANY, 60,
                    IPS_IDLE);

    DerotateDirSP[0].fill("DEROTATE_NORMAL", "Normal", ISS_ON);
    DerotateDirSP[1].fill("DEROTATE_REVERSED", "Reversed", ISS_OFF);
    DerotateDirSP.fill(getDeviceName(), "DEROTATION_DIRECTION", "Direction", DEROTATE_TAB, IP_RW, ISR_1OFMANY, 60,
                       IPS_IDLE);
    DerotateDirSP.load();

    DerotateSettingsNP[DEROT_TRIM].fill("TRIM", "Speed trim (%)", "%.2f", -10, 10, 0.1, 0);
    DerotateSettingsNP[DEROT_MAXRATE].fill("MAX_RATE", "Max rate (deg/s)", "%.3f", 0.005, 0.2, 0.005, 0.05);
    DerotateSettingsNP[DEROT_INTERVAL].fill("INTERVAL", "Update interval (s)", "%.0f", 3, 60, 1, 5);
    DerotateSettingsNP.fill(getDeviceName(), "DEROTATION_SETTINGS", "Settings", DEROTATE_TAB, IP_RW, 60, IPS_IDLE);
    DerotateSettingsNP.load();

    DerotateStatusNP[DEROT_AZ].fill("AZ", "Azimuth (deg)", "%.2f", 0, 360, 0, 0);
    DerotateStatusNP[DEROT_ALT].fill("ALT", "Altitude (deg)", "%.2f", -90, 90, 0, 0);
    DerotateStatusNP[DEROT_RATE].fill("RATE", "Field rotation (deg/h)", "%.2f", -10000, 10000, 0, 0);
    DerotateStatusNP.fill(getDeviceName(), "DEROTATION_STATUS", "Status", DEROTATE_TAB, IP_RO, 60, IPS_IDLE);

    DerotateMountTP[0].fill("MOUNT", "Mount device", "");
    DerotateMountTP.fill(getDeviceName(), "DEROTATION_MOUNT", "Mount to follow", DEROTATE_TAB, IP_RW, 60, IPS_IDLE);
    DerotateMountTP.load();

    serialConnection->setDefaultBaudRate(Connection::Serial::B_19200);


    return true;
}

bool WandererRotatorBase::updateProperties()
{
    INDI::Rotator::updateProperties();

    if (isConnected())
    {
        defineProperty(SetZeroSP);
        defineProperty(BacklashNP);
        defineProperty(AccuracyNP);

        defineProperty(DerotateSP);
        defineProperty(DerotateDirSP);
        defineProperty(DerotateSettingsNP);
        defineProperty(DerotateStatusNP);
        defineProperty(DerotateMountTP);
        applyDerotationSettings();
        derotationSubscribe();
    }
    else
    {
        m_Derotator.stop();
        m_resumeDerotation = m_hasPendingTarget = m_derotMoveInFlight = false;
        deleteProperty(DerotateSP);
        deleteProperty(DerotateDirSP);
        deleteProperty(DerotateSettingsNP);
        deleteProperty(DerotateStatusNP);
        deleteProperty(DerotateMountTP);
        deleteProperty(SetZeroSP);
        deleteProperty(BacklashNP);
        deleteProperty(AccuracyNP);
    }
    return true;
}

bool WandererRotatorBase::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{

    if (dev && !strcmp(dev, getDeviceName()))
    {
        if (SetZeroSP.isNameMatch(name))
        {
            SetZeroSP.setState(sendCommand("1500002") ? IPS_OK : IPS_ALERT);
            SetZeroSP.apply();
            m_signedAngle = 0;
            GotoRotatorNP[0].setValue(0);
            GotoRotatorNP.apply();
            LOG_INFO("Virtual Mechanical Angle is set to zero.");
            return true;
        }
    }
    if (dev && !strcmp(dev, getDeviceName()))
    {
        if (DerotateSP.isNameMatch(name))
        {
            DerotateSP.update(states, names, n);
            const bool wantOn = DerotateSP[0].getState() == ISS_ON;
            if (wantOn && startDerotation())
            {
                DerotateSP.setState(IPS_BUSY);
            }
            else if (wantOn)
            {
                DerotateSP[0].setState(ISS_OFF);
                DerotateSP[1].setState(ISS_ON);
                DerotateSP.setState(IPS_ALERT);
            }
            else
            {
                if (m_Derotator.active())
                    LOG_INFO("Field derotation stopped.");
                m_Derotator.stop();
                m_resumeDerotation = false;
                DerotateSP.setState(IPS_IDLE);
            }
            DerotateSP.apply();
            return true;
        }

        if (DerotateDirSP.isNameMatch(name))
        {
            DerotateDirSP.update(states, names, n);
            DerotateDirSP.setState(IPS_OK);
            DerotateDirSP.apply();
            applyDerotationSettings();
            return true;
        }
    }
    return Rotator::ISNewSwitch(dev, name, states, names, n);
}

bool WandererRotatorBase::ISNewNumber(const char * dev, const char * name, double values[], char * names[], int n)
{
    if (dev && !strcmp(dev, getDeviceName()))
    {
        // backlash
        if (BacklashNP.isNameMatch(name))
        {
            bool rc1 = false;
            BacklashNP.update(values, names, n);
            backlash = BacklashNP[BACKLASH].value;

            char cmd[16];
            snprintf(cmd, 16, "%d", (int)(backlash * 10 + 1600000));
            rc1 = sendCommand(cmd);

            BacklashNP.setState( (rc1) ? IPS_OK : IPS_ALERT);
            if (BacklashNP.getState() == IPS_OK)
                BacklashNP.update(values, names, n);
            BacklashNP.apply();
            LOG_INFO("Backlash Set");
            return true;
        }

        // accuracy
        if (AccuracyNP.isNameMatch(name))
        {
            return updateProperty(AccuracyNP, values, names, n, [this, values]()
            {
                accuracy = values[0];
                return true;
            });
        }

    }
    if (dev && !strcmp(dev, getDeviceName()) && DerotateSettingsNP.isNameMatch(name))
    {
        DerotateSettingsNP.update(values, names, n);
        DerotateSettingsNP.setState(IPS_OK);
        DerotateSettingsNP.apply();
        applyDerotationSettings();
        return true;
    }
    return Rotator::ISNewNumber(dev, name, values, names, n);
}

bool WandererRotatorBase::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    if (dev && !strcmp(dev, getDeviceName()) && DerotateMountTP.isNameMatch(name))
    {
        DerotateMountTP.update(texts, names, n);
        DerotateMountTP.setState(IPS_OK);
        DerotateMountTP.apply();
        m_hasRA = m_hasDec = m_hasLat = m_hasLong = false;
        derotationSubscribe();
        return true;
    }
    return Rotator::ISNewText(dev, name, texts, names, n);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// Field derotation
//////////////////////////////////////////////////////////////////////////////////////////////
double WandererRotatorBase::monotonicSeconds()
{
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now().time_since_epoch()).count();
}

void WandererRotatorBase::derotationSubscribe()
{
    const char *mount = DerotateMountTP[0].getText();
    if (!mount || !*mount)
        return;
    IDSnoopDevice(mount, "EQUATORIAL_EOD_COORD");
    IDSnoopDevice(mount, "GEOGRAPHIC_COORD");
    LOGF_INFO("Derotation: following mount '%s'.", mount);
}

bool WandererRotatorBase::ISSnoopDevice(XMLEle *root)
{
    const char *mount = DerotateMountTP[0].getText();
    const char *dev   = findXMLAttValu(root, "device");
    const char *name  = findXMLAttValu(root, "name");
    const char *tag   = tagXMLEle(root);

    if (mount && *mount && dev && name && tag && !strcmp(dev, mount)
            && (!strcmp(tag, "setNumberVector") || !strcmp(tag, "defNumberVector")))
    {
        const bool isEq  = !strcmp(name, "EQUATORIAL_EOD_COORD");
        const bool isGeo = !strcmp(name, "GEOGRAPHIC_COORD");
        if (isEq || isGeo)
        {
            for (XMLEle *ep = nextXMLEle(root, 1); ep; ep = nextXMLEle(root, 0))
            {
                const char *en = findXMLAttValu(ep, "name");
                double v = 0;
                if (!en || f_scansexa(pcdataXMLEle(ep), &v) != 0)
                    continue;
                if (isEq && !strcmp(en, "RA"))
                {
                    m_mountRA = v;
                    m_hasRA = true;
                }
                else if (isEq && !strcmp(en, "DEC"))
                {
                    m_mountDec = v;
                    m_hasDec = true;
                }
                else if (isGeo && !strcmp(en, "LAT"))
                {
                    m_mountLat = v;
                    m_hasLat = true;
                }
                else if (isGeo && !strcmp(en, "LONG"))
                {
                    m_mountLong = v;
                    m_hasLong = true;
                }
            }
        }
    }
    return INDI::Rotator::ISSnoopDevice(root);
}

void WandererRotatorBase::applyDerotationSettings()
{
    m_Derotator.setTrimPercent(DerotateSettingsNP[DEROT_TRIM].getValue());
    m_Derotator.setMaxRate(DerotateSettingsNP[DEROT_MAXRATE].getValue());
    m_Derotator.setReverse(DerotateDirSP[1].getState() == ISS_ON);
    m_Derotator.setMinStep(1.0 / std::max(1, getStepsPerDegree()));   // smallest possible move: 1 step
}

bool WandererRotatorBase::startDerotation(bool resume)
{
    if (!(m_hasRA && m_hasDec && m_hasLat && m_hasLong))
    {
        LOGF_ERROR("No position data from mount '%s' (need EQUATORIAL_EOD_COORD and GEOGRAPHIC_COORD). "
                   "Set the mount device name first.", DerotateMountTP[0].getText());
        return false;
    }
    applyDerotationSettings();
    m_Derotator.start(0.0, monotonicSeconds());
    m_lastDerotMove = monotonicSeconds();
    m_warnedNoData  = false;
    LOG_INFO(resume ? "Field derotation resumed (integrator reset)." : "Field derotation started.");
    return true;
}

bool WandererRotatorBase::derotationTick()
{
    if (!m_Derotator.active())
        return false;

    if (!(m_hasRA && m_hasDec && m_hasLat && m_hasLong))
    {
        if (!m_warnedNoData)
        {
            LOG_WARN("Derotation waiting for mount position data...");
            m_warnedNoData = true;
        }
        return false;
    }

    const double now = monotonicSeconds();
    double az = 0, alt = 0;
    FieldDerotation::azAltFromRaDec(m_mountRA, m_mountDec, m_mountLat, m_mountLong,
                                    FieldDerotation::julianDateNow(), az, alt);

    double delta = 0;
    const bool due = m_Derotator.stepRelative(az, alt, m_mountLat, now, delta);

    DerotateStatusNP[DEROT_AZ].setValue(az);
    DerotateStatusNP[DEROT_ALT].setValue(alt);
    DerotateStatusNP[DEROT_RATE].setValue(m_Derotator.lastRate() * 3600.0);
    DerotateStatusNP.setState(m_Derotator.rateLimited() ? IPS_ALERT : IPS_OK);
    DerotateStatusNP.apply();

    if (!due || now - m_lastDerotMove < DerotateSettingsNP[DEROT_INTERVAL].getValue())
        return false;

    const int spd   = getStepsPerDegree();
    const int steps = static_cast<int>(std::lround(delta * spd));
    if (steps == 0)
        return false;

    const double quantized = steps / static_cast<double>(spd);
    char cmd[16];
    snprintf(cmd, 16, "%d", 1000000 + steps);    // relative move, same protocol as MoveRotator()
    positionhistory = quantized;
    if (!Move(cmd))
        return false;

    m_derotMoveInFlight = true;
    m_Derotator.consumed(quantized);
    m_lastDerotMove = now;
    GotoRotatorNP.setState(IPS_BUSY);
    GotoRotatorNP.apply();
    return true;
}

bool WandererRotatorBase::saveConfigItems(FILE *fp)
{
    INDI::Rotator::saveConfigItems(fp);
    AccuracyNP.save(fp);
    DerotateSettingsNP.save(fp);
    DerotateDirSP.save(fp);
    DerotateMountTP.save(fp);
    return true;
}

bool WandererRotatorBase::Handshake()
{
    if (isSimulation())
    {
        LOG_INFO("Simulation mode: no serial communication with the rotator.");
        firmware = getMinimumCompatibleFirmwareVersion();
        M_angleread = 0;
        m_simAngle = 0;
        m_signedAngle = 0;
        GotoRotatorNP[0].setValue(0);
        BacklashNP[BACKLASH].setValue(0);
        BacklashNP.setState(IPS_OK);
        BacklashNP.apply();
        return true;
    }

    PortFD = serialConnection->getPortFD();
    tcflush(PortFD, TCIOFLUSH);
    int nbytes_read_name = 0, nbytes_written = 0, rc = -1;
    char name[64] = {0};

    if ((rc = tty_write_string(PortFD, "1500001", &nbytes_written)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Serial write error: %s", errorMessage);
        return false;
    }

    //Device Model//////////////////////////////////////////////////////////////////////////////////////////////////////
    if ((rc = tty_read_section(PortFD, name, 'A', 3, &nbytes_read_name)) != TTY_OK)
    {
        tcflush(PortFD, TCIOFLUSH);
        if ((rc = tty_write_string(PortFD, "1500001", &nbytes_written)) != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(rc, errorMessage, MAXRBUF);
            LOGF_ERROR("Serial write error: %s", errorMessage);
            return false;
        }
        if ((rc = tty_read_section(PortFD, name, 'A', 3, &nbytes_read_name)) != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(rc, errorMessage, MAXRBUF);
            LOG_INFO("No data received, the device may not be WandererRotator, please check the serial port!");
            LOGF_ERROR("Device read error: %s", errorMessage);
            return false;
        }
    }
    name[nbytes_read_name - 1] = '\0';
    if(strcmp(name, getRotatorHandshakeName()) != 0)
    {
        LOGF_ERROR("The device is not %s", getDefaultName());
        LOGF_INFO("The device is %s.", name);
        return false;
    }
    // Frimware version/////////////////////////////////////////////////////////////////////////////////////////////
    int nbytes_read_version = 0;
    char version[64] = {0};
    tty_read_section(PortFD, version, 'A', 5, &nbytes_read_version);

    version[nbytes_read_version - 1] = '\0';
    LOGF_INFO("Firmware Version:%s", version);
    firmware = std::atoi(version);
    if(firmware < getMinimumCompatibleFirmwareVersion())
    {
        LOG_ERROR("The firmware is outdated, please upgrade to the latest firmware!");
        LOGF_ERROR("The current firmware is %d.", firmware);
        return false;
    }

    // Angle//////////////////////////////////////////////////////////////////////////////////////////
    char M_angle[64] = {0};
    int nbytes_read_M_angle = 0;
    tty_read_section(PortFD, M_angle, 'A', 5, &nbytes_read_M_angle);
    M_angle[nbytes_read_M_angle - 1] = '\0';
    M_angleread = std::strtod(M_angle, NULL);

    if(std::fabs(M_angleread) > 400000)
    {
        rc = sendCommand("1500002");
        LOG_WARN("Virtual Mechanical Angle is too large, it is now set to zero!");
        rc = sendCommand("1500001");
        rc = tty_read_section(PortFD, M_angle, 'A', 5, &nbytes_read_M_angle);
        rc = tty_read_section(PortFD, M_angle, 'A', 5, &nbytes_read_M_angle);
        rc = tty_read_section(PortFD, M_angle, 'A', 5, &nbytes_read_M_angle);
        M_angle[nbytes_read_M_angle - 1] = '\0';
        M_angleread = std::strtod(M_angle, NULL);
    }
    m_signedAngle = M_angleread / 1000;
    GotoRotatorNP[0].setValue(FieldDerotation::wrap360(m_signedAngle));
    //backlash/////////////////////////////////////////////////////////////////////
    char M_backlash[64] = {0};
    int nbytes_read_M_backlash = 0;
    tty_read_section(PortFD, M_backlash, 'A', 5, &nbytes_read_M_backlash);
    M_backlash[nbytes_read_M_backlash - 1] = '\0';
    M_backlashread = std::strtod(M_backlash, NULL);

    BacklashNP[BACKLASH].setValue(M_backlashread);
    BacklashNP.setState(IPS_OK);
    BacklashNP.apply();
    //reverse/////////////////////////////////////////////////////////////////////
    char M_reverse[64] = {0};
    int nbytes_read_M_reverse = 0;
    tty_read_section(PortFD, M_reverse, 'A', 5, &nbytes_read_M_reverse);
    M_reverse[nbytes_read_M_reverse - 1] = '\0';
    M_reverseread = std::strtod(M_reverse, NULL);
    if(M_reverseread == 0)
    {
        ReverseRotator(false);
    }
    else
    {
        ReverseRotator(true);
    }

    tcflush(PortFD, TCIOFLUSH);
    return true;
}


IPState WandererRotatorBase::MoveRotator(double angle)
{
    return requestAbsoluteMove(angle, false);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// A client (e.g. the Ekos sequencer) wants a new absolute angle. Derotation is paused and its
/// integrator is discarded; it is resumed (integrator = 0) when the move is finished.
//////////////////////////////////////////////////////////////////////////////////////////////
IPState WandererRotatorBase::requestAbsoluteMove(double angle, bool exact)
{
    if (m_Derotator.active())
    {
        m_Derotator.stop();
        m_resumeDerotation = true;
        DerotateSP.setState(IPS_OK);       // enabled, but paused
        DerotateSP.apply();
        LOG_INFO("Field derotation paused for rotator move.");
    }

    // A derotation step still waits for the firmware's reply: do not send a second command now
    // (the replies would get mixed up), queue the target instead. TimerHit starts it afterwards.
    if (m_derotMoveInFlight)
    {
        m_pendingTarget = angle;
        m_pendingExact = exact;
        m_hasPendingTarget = true;
        return IPS_BUSY;
    }

    IPState st = startAbsoluteMove(angle, exact);
    if (st != IPS_BUSY)
        resumeDerotation();
    return st;
}

IPState WandererRotatorBase::startAbsoluteMove(double angle, bool exact)
{
    if (accuracy > 0)
        angle = std::round(angle / accuracy) * accuracy;

    // The signed virtual angle can be negative after derotation, so work relative to it.
    // Client moves take the shortest way (angles are modulo 360), "exact" moves (Home) go straight to the value.
    const double delta = exact ? (angle - m_signedAngle) : wrap180Deg(angle - m_signedAngle);

    // Gate on the actual step delta the firmware will see. Anything that
    // rounds to zero steps is a no-op the firmware won't acknowledge, so
    // return immediately instead of blocking in TimerHit ("not powered").
    const int spd = getStepsPerDegree();
    const int steps = static_cast<int>(std::round(delta * spd));
    if (steps == 0)
        return IPS_OK;

    char cmd[16];
    positionhistory = steps / static_cast<double>(spd);
    snprintf(cmd, 16, "%d", steps + 1000000);
    if (!Move(cmd))
        return IPS_ALERT;

    return IPS_BUSY;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// Called when a move has been acknowledged (or failed/aborted). Starts a queued client move.
/// \return true if another move was started (the property must stay BUSY)
//////////////////////////////////////////////////////////////////////////////////////////////
bool WandererRotatorBase::continueAfterMove(bool halted)
{
    m_derotMoveInFlight = false;
    if (!m_hasPendingTarget)
        return false;
    m_hasPendingTarget = false;
    if (halted)
        return false;

    if (startAbsoluteMove(m_pendingTarget, m_pendingExact) == IPS_BUSY)
    {
        GotoRotatorNP.apply();             // publish the intermediate angle, state stays BUSY
        return true;
    }
    return false;
}

void WandererRotatorBase::resumeDerotation()
{
    if (!m_resumeDerotation)
        return;
    m_resumeDerotation = false;
    if (startDerotation(true))
    {
        DerotateSP.setState(IPS_BUSY);
    }
    else
    {
        DerotateSP[0].setState(ISS_OFF);
        DerotateSP[1].setState(ISS_ON);
        DerotateSP.setState(IPS_ALERT);
    }
    DerotateSP.apply();
}


bool WandererRotatorBase::AbortRotator()
{


    if (GotoRotatorNP.getState() == IPS_BUSY)
    {
        haltcommand = true;
        int nbytes_written = 0, rc = -1;
        if (!isSimulation())
            tcflush(PortFD, TCIOFLUSH);
        if (!isSimulation() && (rc = tty_write_string(PortFD, "Stop", &nbytes_written)) != TTY_OK)
        {
            char errorMessage[MAXRBUF];
            tty_error_msg(rc, errorMessage, MAXRBUF);
            LOGF_ERROR("Serial write error: %s", errorMessage);
            return false;
        }
        SetTimer(100);
    }
    return true;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief WandererRotatorBase::HomeRotator
/// \return
///
IPState WandererRotatorBase::HomeRotator()
{
    if (std::fabs(m_signedAngle) * getStepsPerDegree() >= 0.5)
    {
        LOG_INFO("Moving to zero...");
        GotoRotatorNP.setState(IPS_BUSY);
        requestAbsoluteMove(0.0, true);
    }
    return IPS_OK;
}


bool WandererRotatorBase::ReverseRotator(bool enabled)
{

    if (enabled)
    {
        char cmd[16];
        snprintf(cmd, 16, "%d", 1700001);
        if (sendCommand(cmd) != true)
        {
            LOG_ERROR("Serial write error.");
            return false;
        }
        ReverseState = true;
        return true;
    }
    else
    {
        char cmd[16];
        snprintf(cmd, 16, "%d", 1700000);
        if (sendCommand(cmd) != true)
        {
            LOG_ERROR("Serial write error.");
            return false;
        }
        ReverseState = false;
        return true;
    }
    return false;
}



void WandererRotatorBase::TimerHit()
{
    const bool wasBusy = (GotoRotatorNP.getState() == IPS_BUSY || haltcommand == true);
    bool continued = false;   // a queued client move was started in this tick (it armed the timer itself)

    if (GotoRotatorNP.getState() == IPS_BUSY || haltcommand == true)
    {

        if(nowtime < estime && haltcommand == false)
        {
            GotoRotatorNP[0].setValue(GotoRotatorNP[0].getValue() + std::copysign(1.0, positionhistory));
            GotoRotatorNP.apply();
            nowtime = nowtime + 240;
            SetTimer(240);
            return;
        }
        else
        {
            int rc = -1;
            estime = 0;
            nowtime = 0;
            if (isSimulation())
            {
                const bool halted = haltcommand;
                if (!halted)
                    m_simAngle += positionhistory;
                m_signedAngle = m_simAngle;
                GotoRotatorNP[0].setValue(FieldDerotation::wrap360(m_signedAngle));
                haltcommand = false;
                if (continueAfterMove(halted))
                {
                    continued = true;
                }
                else
                {
                    GotoRotatorNP.setState(IPS_OK);
                    GotoRotatorNP.apply();
                    resumeDerotation();
                }
            }
            else
            {
                char M_angle[64] = {0};
                int nbytes_read_M_angle = 0;
                rc = tty_read_section(PortFD, M_angle, 'A', 5, &nbytes_read_M_angle);
                if(rc != TTY_OK)
                {
                    LOG_ERROR("Rotator not powered!");
                    GotoRotatorNP[0].setValue(initangle);
                    GotoRotatorNP.setState(IPS_ALERT);   // do not stay BUSY forever
                    GotoRotatorNP.apply();
                    haltcommand = false;
                    continueAfterMove(true);             // drop queued client move, clear in-flight flag
                    resumeDerotation();
                    SetTimer(2000);                      // keep the timer (and derotation) alive
                    return;
                }
                tty_read_section(PortFD, M_angle, 'A', 5, &nbytes_read_M_angle);

                M_angle[nbytes_read_M_angle - 1] = '\0';
                M_angleread = std::strtod(M_angle, NULL);
                m_signedAngle = M_angleread / 1000;
                GotoRotatorNP[0].setValue(FieldDerotation::wrap360(m_signedAngle));
                const bool halted = haltcommand;
                haltcommand = false;
                if (continueAfterMove(halted))
                {
                    continued = true;
                }
                else
                {
                    GotoRotatorNP.setState(IPS_OK);
                    GotoRotatorNP.apply();
                    resumeDerotation();
                }
            }
        }
    }

    // Field derotation: only when the rotator was idle at the start of this tick.
    // A move issued by derotationTick() already armed the timer inside Move().
    bool moved = false;
    if (!wasBusy)
        moved = derotationTick();

    if (!moved && !continued)
        SetTimer(m_Derotator.active() ? 1000 : 2000);
}



bool WandererRotatorBase::Move(const char *cmd)
{
    initangle = GotoRotatorNP[0].getValue();
    int nbytes_written = 0, rc = -1;
    LOGF_DEBUG("CMD <%s>", cmd);
    if (!isSimulation() && (rc = tty_write_string(PortFD, cmd, &nbytes_written)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Serial write error: %s", errorMessage);
        return false;
    }
    SetTimer(2000);
    nowtime = 0;
    estime = abs((std::atoi(cmd) - 1000000) / getStepsPerDegree() * 240);
    return true;
}


bool WandererRotatorBase::sendCommand(std::string command)
{
    int nbytes_written = 0, rc = -1;
    std::string command_termination = "\n";
    LOGF_DEBUG("CMD: %s", command.c_str());
    if (!isSimulation() && (rc = tty_write_string(PortFD, (command + command_termination).c_str(), &nbytes_written)) != TTY_OK)
    {
        char errorMessage[MAXRBUF];
        tty_error_msg(rc, errorMessage, MAXRBUF);
        LOGF_ERROR("Serial write error: %s", errorMessage);
        return false;
    }
    return true;
}
