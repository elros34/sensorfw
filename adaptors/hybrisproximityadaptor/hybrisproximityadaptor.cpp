/****************************************************************************
**
** Copyright (C) 2013 Jolla Ltd
**
**
** $QT_BEGIN_LICENSE:LGPL$
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include <QDebug>
#include <QFile>
#include <QTextStream>

#include "hybrisproximityadaptor.h"
#include "logging.h"
#include "datatypes/utils.h"
#include "config.h"
#include <fcntl.h>
#include <unistd.h>

HybrisProximityAdaptor::HybrisProximityAdaptor(const QString& id) :
    HybrisAdaptor(id,SENSOR_TYPE_PROXIMITY),
    lastNearValue(-1)
{
    if (isValid()) {
        buffer = new DeviceAdaptorRingBuffer<ProximityData>(1);
        setAdaptedSensor("proximity", "Internal proximity coordinates", buffer);

        setDescription("Hybris proximity");
        powerStatePath = SensorFrameworkConfig::configuration()->value("proximity/powerstate_path").toByteArray();
	if (!powerStatePath.isEmpty() && !QFile::exists(powerStatePath))
	{
	    sensordLogW() << "Path does not exists: " << powerStatePath;
	    powerStatePath.clear();
	}
    }
}

HybrisProximityAdaptor::~HybrisProximityAdaptor()
{
    if (isValid()) {
        delete buffer;
    }
}

bool HybrisProximityAdaptor::startSensor()
{
    if (!(HybrisAdaptor::startSensor()))
        return false;
    if (!powerStatePath.isEmpty())
        writeToFile(powerStatePath, "1");
    sensordLogD() << "HybrisProximityAdaptor start\n";
    return true;
}

void HybrisProximityAdaptor::sendInitialData()
{
   ProximityData *d = buffer->nextSlot();

   d->timestamp_ = Utils::getTimeStamp();
   d->withinProximity_ = false;
   d->value_ = maxRange();

   buffer->commit();
   buffer->wakeUpReaders();

   if (!powerStatePath.isEmpty())
       writeToFile(powerStatePath, "1"); // writting 1 will trigger data change
}

void HybrisProximityAdaptor::stopSensor()
{
    HybrisAdaptor::stopSensor();
    if (!powerStatePath.isEmpty())
        writeToFile(powerStatePath, "0");
    sensordLogD() << "HybrisProximityAdaptor stop\n";
}

void HybrisProximityAdaptor::processSample(const sensors_event_t& data)
{
    ProximityData *d = buffer->nextSlot();
    d->timestamp_ = quint64(data.timestamp * .001);
    bool near = false;
#ifdef USE_BINDER
    if (data.u.scalar < maxRange()) {
        near = true;
    }
    d->value_ = data.u.scalar;
#else
    if (data.distance < maxRange()) {
        near = true;
    }
    d->value_ = data.distance;
#endif
    d->withinProximity_ = near;

    lastNearValue = near;
    buffer->commit();
    buffer->wakeUpReaders();
}
