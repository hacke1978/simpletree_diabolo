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

#ifndef ST_PLUGINMANAGER_H
#define ST_PLUGINMANAGER_H

#include "ct_abstractstepplugin.h"

#include "ct_log/ct_fileloglistener.h"

class ST_PluginManager : public CT_AbstractStepPlugin
{
public:
    ST_PluginManager();
    ~ST_PluginManager();

    QString getPluginURL() const {return QString("http://rdinnovation.onf.fr/projects/PLUGINS-PROJECT-NAME-HERE/wiki");}

    QString getPluginOfficialName() const;
    QStringList getPluginRISCitationList() const;

protected:

    bool loadGenericsStep();
    bool loadOpenFileStep();
    bool loadCanBeAddedFirstStep();
    bool loadActions();
    bool loadExporters();
    bool loadReaders();

private:
    CT_FileLogListener  m_logListener;
};

#endif // ST_PLUGINMANAGER_H
