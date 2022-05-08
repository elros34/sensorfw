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

#include <QFile>
#include <QTextStream>

#include "hybrisalsadaptor.h"
#include "logging.h"
#include "datatypes/utils.h"
#include "config.h"

#include <fcntl.h>
#include <unistd.h>

HybrisAlsAdaptor::HybrisAlsAdaptor(const QString& id) :
    HybrisAdaptor(id,SENSOR_TYPE_LIGHT),
    lastLightValue(9999)
{
    buffer = new DeviceAdaptorRingBuffer<TimedUnsigned>(1);
    setAdaptedSensor("als", "Internal ambient light sensor lux values", buffer);
    setDescription("Hybris als");
    powerStatePath = SensorFrameworkConfig::configuration()->value("als/powerstate_path").toByteArray();
    if (!powerStatePath.isEmpty() && !QFile::exists(powerStatePath))
    {
    	sensordLogW() << "Path does not exists: " << powerStatePath;
    	powerStatePath.clear();
    }
}

HybrisAlsAdaptor::~HybrisAlsAdaptor()
{
    delete buffer;
}

bool HybrisAlsAdaptor::startSensor()
{
    if (!(HybrisAdaptor::startSensor()))
        return false;
    if (!powerStatePath.isEmpty())
        writeToFile(powerStatePath, "1");
    sensordLogD() << "Hybris HybrisAlsAdaptor start\n";
    return true;
}

void HybrisAlsAdaptor::sendInitialData()
{
    TimedUnsigned *d = buffer->nextSlot();
    d->timestamp_ = Utils::getTimeStamp();
    d->value_ = lastLightValue - 1; // workaround for qtsensors
    buffer->commit();
    buffer->wakeUpReaders();

    if (!powerStatePath.isEmpty())
        writeToFile(powerStatePath, "1"); // writting 1 will trigger data change
}

void HybrisAlsAdaptor::stopSensor()
{
    HybrisAdaptor::stopSensor();
    if (!powerStatePath.isEmpty())
        writeToFile(powerStatePath, "0");
    sensordLogD() << "Hybris HybrisAlsAdaptor stop\n";
}

void HybrisAlsAdaptor::processSample(const sensors_event_t& data)
{
    TimedUnsigned *d = buffer->nextSlot();
    d->timestamp_ = quint64(data.timestamp * .001);
#ifdef USE_BINDER
    d->value_ = data.u.scalar;
#else
    d->value_ = data.light;
#endif
    lastLightValue = d->value_;
    buffer->commit();
    buffer->wakeUpReaders();
}
