# -*- coding: utf-8 -*-
"""
Created on Thu Jul  4 11:00:19 2019

@author: erazer
"""

import ogr
import os
import numpy as np
import csv

#############
# List files
LSlist = []
DSlist = []
statisticsLine = []
files = []

digitizedFiles = os.listdir(os.path.join(os.getcwd(),'data','Digitized Route Shape File'))
for f in digitizedFiles:
    if f.endswith(".shp"):
        files.append(f.split('.')[0])

for aname in files:
    # reading a shapefile
    in_path = os.path.join(os.getcwd(),'data','Digitized Route Shape File',aname + '.shp')
    
    # get the correct driver
    driver = ogr.GetDriverByName('ESRI Shapefile')
    
    # 0 means read-only. 1 means writeable.
    data_source = driver.Open(in_path, 0) 
    
    # Check to see if shapefile is found.
    if data_source is None:
        print('Could not open %s' % (in_path))
    
    #print('Opened %s' % (in_path))
    # get the Layer class object
    layer = data_source.GetLayer(0)
    
    # get reference system info
    spatial_ref = layer.GetSpatialRef()
    
    # get info about the attributes (fields)
    attributes = layer.GetLayerDefn()
    #for i in range(attributes.GetFieldCount()):
        #print(attributes.GetFieldDefn(i).GetName())
    
    # get info about the features
    feature_count = layer.GetFeatureCount()
    print("Number of features in %s: %d" % \
          (os.path.basename(in_path),feature_count))
    
    # Calculate statistics
    #access single features
    
    
    for feat in layer:
        pt = feat.geometry()
        x = pt.GetX()
        y = pt.GetY()
        
        LS = feat.GetField('LS')
        DS = feat.GetField('DS')
        
        LSlist.append(LS)
        DSlist.append(DS)
        
    statisticsLine.append([aname, np.min(LSlist), np.max(LSlist), np.sum(LSlist), np.mean(LSlist), np.std(LSlist), np.var(LSlist), np.median(LSlist),\
              np.min(DSlist), np.max(DSlist), np.sum(DSlist), np.mean(DSlist), np.std(DSlist), np.var(DSlist), np.median(DSlist)])
    LSlist = []
    DSlist = []

# calculate statistics!

# Set up the table
statisticsHead = ['file', 'LSmin','LSmax','LSsum', 'LSmean', 'LSstandardDev','LSvarience', 'LSmedian','DSmin','DSmax','DSsum', 'DSmean', 'DSsd','DSvarience', 'DSmedian']
# Write down the table
with open('AStar_observed_landmark_metrics.csv', 'w',newline='') as fp:
   writer = csv.writer(fp, delimiter=';')
   writer.writerow(statisticsHead)
   for l in range(0,len(statisticsLine)):
       print(statisticsLine[l])
       writer.writerow(statisticsLine[l])
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    