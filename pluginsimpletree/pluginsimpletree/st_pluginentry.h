/****************************************************************************

 Copyright (C) 2016-2017 INRA (Institut National de la Recherche Agronomique, France) and IGN (Institut National de l'information Géographique et forestière, France)
 All rights reserved.

 Contact : jan.hackenberg@posteo.de

 Developers : Jan Hackenberg

 This file is part of Simpletree plugin Version 4 for Computree.

 Simpletree plugin is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Simpletree plugin is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Simpletree plugin.  If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/

#ifndef ST_PLUGINENTRY_H
#define ST_PLUGINENTRY_H

#include "interfaces.h"

class ST_PluginManager;

class ST_PluginEntry : public PluginEntryInterface
{
    Q_OBJECT

#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
    Q_PLUGIN_METADATA(IID PluginEntryInterface_iid)
#endif

    Q_INTERFACES(PluginEntryInterface)

public:
    ST_PluginEntry();
    ~ST_PluginEntry();

    QString getVersion() const;
    CT_AbstractStepPlugin* getPlugin() const;

private:
    ST_PluginManager *_pluginManager;
};

#endif // ST_PLUGINENTRY_H
