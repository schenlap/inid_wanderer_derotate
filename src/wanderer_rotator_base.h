/*******************************************************************************
  Copyright(c) 2025 Frank Wang & Jérémie Klein. All rights reserved.

  WandererRotator Base

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

#pragma once

#include "defaultdevice.h"
#include "indirotator.h"
#include "indirotatorinterface.h"
#include "indipropertyswitch.h"
#include "indipropertynumber.h"
#include "indipropertytext.h"
#include "lilxml.h"
#include "field_derotator.h"

class WandererRotatorBase : public INDI::Rotator
{
public:
    WandererRotatorBase();

    bool initProperties() override;
    bool updateProperties() override;
    bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;
    bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) override;
    bool ISSnoopDevice(XMLEle *root) override;
    bool saveConfigItems(FILE *fp) override;

protected:
    virtual const char * getDefaultName() override = 0;
    virtual const char * getRotatorHandshakeName() = 0;
    virtual int getMinimumCompatibleFirmwareVersion() = 0;
    virtual int getStepsPerDegree() = 0;

    IPState MoveRotator(double angle) override;
    IPState HomeRotator() override;
    bool ReverseRotator(bool enabled) override;
    bool AbortRotator() override;
    void TimerHit() override;

    bool Handshake() override;
    bool sendCommand(std::string command);
    bool Move(const char *cmd);

    // ---- Field derotation for alt-az mounts (follows a mount via INDI snooping) ----
    bool derotationTick();            // returns true if a move was issued
    void derotationSubscribe();
    void applyDerotationSettings();
    bool startDerotation(bool resume = false);

    // ---- Client moves (e.g. Ekos sequencer) while derotation is active ----
    IPState requestAbsoluteMove(double angle, bool exact = false);  // pauses derotation, queues behind a running derotation step
    IPState startAbsoluteMove(double angle, bool exact);            // issues the relative step command for an absolute target
    bool continueAfterMove(bool halted);                            // true if a queued target was started (state stays BUSY)
    void resumeDerotation();                                        // restarts derotation (integrator = 0) if it was active before
    static double monotonicSeconds();

    INDI::PropertySwitch DerotateSP{2};            // Enable / Disable
    INDI::PropertySwitch DerotateDirSP{2};         // Normal / Reversed
    INDI::PropertyNumber DerotateSettingsNP{3};    // trim [%], max rate [deg/s], interval [s]
    INDI::PropertyNumber DerotateStatusNP{4};      // az, alt, field rotation rate [deg/h], rotator speed [deg/h] (read only)
    INDI::PropertyText   DerotateMountTP{1};       // INDI device name of the mount to follow

    FieldDerotation::Derotator m_Derotator;
    double m_mountRA = 0, m_mountDec = 0, m_mountLat = 0, m_mountLong = 0;
    bool m_hasRA = false, m_hasDec = false, m_hasLat = false, m_hasLong = false;
    bool m_warnedNoData = false;
    double m_lastDerotMove = 0;
    double m_simAngle = 0;          // mechanical angle in simulation mode
    double m_signedAngle = 0;       // virtual mechanical angle reported by the firmware (signed, deg)
    bool m_derotMoveInFlight = false;  // a derotation step still waits for its acknowledgement
    bool m_resumeDerotation = false;   // derotation was running when a client move was requested
    bool m_hasPendingTarget = false;   // client move queued until the running derotation step is done
    bool m_pendingExact = false;       // queued target is an exact (non-shortest-path) move, e.g. Home
    double m_pendingTarget = 0;

    enum { DEROT_TRIM, DEROT_MAXRATE, DEROT_INTERVAL };
    enum { DEROT_AZ, DEROT_ALT, DEROT_RATE, DEROT_SPEED };

    INDI::PropertySwitch SetZeroSP{1};
    INDI::PropertyNumber BacklashNP{1};
    INDI::PropertyNumber AccuracyNP{1};

    int firmware = 0;
    double M_angleread = 0;
    double M_backlashread = 0;
    double M_reverseread = 0;
    double initangle = 0;
    bool haltcommand = false;
    bool ReverseState = false;
    double backlash = 0.5;
    double accuracy = 0.0;
    double positionhistory = 0;
    int estime = 0;
    int nowtime = 0;

    enum
    {
        BACKLASH,
    };
};
