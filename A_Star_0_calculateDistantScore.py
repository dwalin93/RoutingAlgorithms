# -*- coding: utf-8 -*-
"""

Script to read the distant score from the visibility analysis.
Output needs to be post-processed in GIS

Created on Sun Jun 30 16:23:46 2019

Author: Katarzyna Goch

"""
import os
from osgeo import ogr
import json

# Calculate the distant score fot the nodes.

# Import sightlines and buildings with the global score.
buildingsGS = buildingGlobalScoreFromShp(r"data_visibility/Buildings_Aassee_Scores.shp")
# Data for the OD pair no 1:
nodesWithGS = nodeDSfromShp(r"data_visibility/sightlines/Destination1.shp",buildingsGS)
driver = ogr.GetDriverByName('ESRI Shapefile')

def buildingGlobalScoreFromShp(buildingsDS):
    # dictionairy where key = building ID and value = GS
    buildingGlobalScoreDict = {}
    buildings = driver.Open(buildingsDS, 0) # 0 means read-only. 1 means writeable.
    
    # Check to see if shapefile is found.
    if buildings is None:
        print('Could not open %s' % (buildingsDS))
    else:
        print('Opened %s' % (buildingsDS))
        buildingsLayer = buildings.GetLayer()
        buildingsCount = buildingsLayer.GetFeatureCount()
        print("Number of features in %s: %d" % (os.path.basename(buildingsDS),buildingsCount))
    
    for b in range(0,buildingsCount):
        #get feature of the shapefile 
        buildingsFeature = buildingsLayer.GetFeature(b)
        f = buildingsFeature.ExportToJson()
        json_data = json.loads(f)
        buildingID = json_data['properties']['buildingID']
        buildingGlobalScore = json_data['properties']['Sgb_Scores']
        buildingGlobalScoreDict[buildingID] = buildingGlobalScore
    return buildingGlobalScoreDict

def nodeDSfromShp(sightlineDS, buildingsGS):
    # list with nodeIDs and buildingIDs
    nodesBuildings = {}
            
    sightlines = driver.Open(sightlineDS, 0) # 0 means read-only. 1 means writeable.
    
    # Check to see if shapefile is found.
    if sightlines is None:
        print('Could not open %s' % (sightlineDS))
    else:
        print('Opened %s' % (sightlineDS))
        sightlinesLayer = sightlines.GetLayer()
        sightlinesCount = sightlinesLayer.GetFeatureCount()
        print("Number of features in %s: %d" % (os.path.basename(sightlineDS),sightlinesCount))
    
    for s in range(0,sightlinesCount):
        #get feature of the shapefile 
        sightlinesFeature = sightlinesLayer.GetFeature(s)
        f = sightlinesFeature.ExportToJson()
        json_data = json.loads(f)
        nodeID = json_data['properties']['nodeID']
        buildingID = json_data['properties']['buildingID']
        buildingGlobalScore = buildingsGS[buildingID]
        if nodeID in nodesBuildings:
            if buildingGlobalScore > nodesBuildings[nodeID]:
                nodesBuildings[nodeID] = buildingGlobalScore
        else:
            nodesBuildings[nodeID] = buildingGlobalScore
    print(nodesBuildings)
    return nodesBuildings

       
with open('newMutaz2.csv', 'w') as f:
    for key in nodesWithGS.keys():
        f.write("%s,%s\n"%(key,nodesWithGS[key]))
