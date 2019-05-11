import os
import csv
from collections import OrderedDict
import matplotlib.pyplot as plt

class Csv(object):
	def __init__(self, file = None):
		self.file = file
  
	def write(self, headers =[], rows = [], mode = 'a'):
		self.headers = headers
		self.rows = rows
		self.mode = mode
		# if os.path.isfile(self.file):
		# 	self.headers = []
		with open(self.file, self.mode) as csvfile:
			writer = csv.writer(csvfile, dialect = 'excel')
			if self.mode == 'a':
				csvfile.seek(0, 2)
			else:
				writer.writerow(self.headers)
			
			for j in range(len(self.rows[0])):
				rows_values = []
				for i in range(len(self.rows)):
					rows_values.append(self.rows[i][j])
				writer.writerow(rows_values)
	
	def read(self):
		data = OrderedDict()
		with open(self.file, 'r') as csvfile:
			csvreader = csv.reader(csvfile)
			headers = next(csvreader)
			for header in headers:
				data[header] = list()
			for row in csvreader:
				for i in range(len(row)):
					data[data.keys()[i]].append(row[i])
				
		return data

	def convert(self, value=[],format=float):
			data = []
			for elem in value:
				data.append(format(elem))
			return data

class Plot(object):
	@staticmethod
	def plot(x_value = None, y_value = None, marker = 'ro',markersize= 5,
	         axis_max = None, axis_min = None,title = None, labels = None,
	         save_path = None, show = False,figure_type ='line',color='red'):
		# fig  = plt.figure()
		# plt.plot(x_value, y_value, marker, markersize = 3)
		# if axis_max:
		# 	plt.axis([0, axis_max['x'], 0, axis_max['y']])
		# if labels:
		# 	try:
		# 		plt.xlabel(labels['x'])
		# 		plt.ylabel(labels['y'])
		# 	except:
		# 		pass
		# if title:
		# 	plt.title(title, loc = 'center')
		# if save_path:
		# 	plt.savefig(save_path)
		# if show:
		# 	plt.show()
		# plt.close()
		fig = plt.figure()
		plt.plot(x_value, y_value,color=color)
		if axis_max:
			plt.axis([axis_min['x'], axis_max['x'], axis_min['y'], axis_max['y']])
		if labels:
				plt.xlabel(labels['x'])
				plt.ylabel(labels['y'])
		
		if save_path:
			plt.savefig(save_path)
		
		if show:
			plt.show()
			
		plt.close(fig)
