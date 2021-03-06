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
#include "st_pluginentry.h"
#include "st_pluginmanager.h"

ST_PluginEntry::ST_PluginEntry()
{
    _pluginManager = new ST_PluginManager();
}

ST_PluginEntry::~ST_PluginEntry()
{
    delete _pluginManager;
}

QString ST_PluginEntry::getVersion() const
{
    return "1.0";
}

CT_AbstractStepPlugin* ST_PluginEntry::getPlugin() const
{
    return _pluginManager;
}

#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
    Q_EXPORT_PLUGIN2(plug_simpletree, ST_PluginEntry)
#endif
