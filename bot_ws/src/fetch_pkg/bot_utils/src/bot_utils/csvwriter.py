#!/usr/bin/env python
import os
import csv

def csvwriter(filepath=None, headers=[], rows = [], mode='w'):
        if os.path.isfile(filepath):
            mode = 'a'
            headers =[]
        with open(filepath,  mode) as csvfile:           
            writer = csv.writer(csvfile, dialect='excel')      
                 
            if not headers: 
                csvfile.seek(0,2)
            else:
                writer.writerow(headers)
          
            for j in range(len(rows[0])):
                rows_values = []
                for i in range(len(rows)):
                    rows_values.append(rows[i][j])       
                writer.writerow(rows_values)
        



    
