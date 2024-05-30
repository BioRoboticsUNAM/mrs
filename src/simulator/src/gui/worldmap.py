#!/usr/bin/env python3

import re

class Polygon:
	def __init__(self, pname, ptype, vtcs=[]):
		self._name = pname
		self._type = ptype
		self._vtcs = vtcs
	#end def


	@property
	def name(self):
		return self._name
	#end def


	@property
	def type(self):
		return self._type
	#end def


	@property
	def vertices(self):
		return self._vtcs
	#end def


	def getScaledVertices(self, sx, sy):
		return [ (v[0]*sx, v[1]*sy) for v in self.vertices ]
	#end def


	def __str__(self):
		vtcs = ', '.join([ f'{v}' for v in self.vertices ])
		return f'{self.name} ({self.type}, {vtcs})'
#end def



class WorldMap:
	def __init__(self):
		self._name   = None
		self._width  = 0
		self._height = 0
		self._poly   = []
	#end def


	@property
	def name(self):
		return self._name
	#end def


	@property
	def width(self):
		return self._width
	#end def


	@property
	def height(self):
		return self._height
	#end def


	@property
	def polygons(self):
		return self._poly
	#end def


	def __str__(self):
		sp = [ str(p) for p in  self.polygons]
		s = f'{self.name} ({self.width}, {self.height}): '
		s+= ' '.join(sp)
		return s
#end class


__reFloat = r'-?\d+(\.\d+)?'
__reId    = r'[A-Za-z_]\w*'
# __reFloat = r'\S+'

__rxDimensions = re.compile(r'\(\s*dimensions\s+' +\
	rf'(?P<roomName>{__reId})\s+' +\
	rf'(?P<roomWidth>{__reFloat})\s+' +\
	rf'(?P<roomHeight>{__reFloat})\s*\)'
)

__rxPolygon = re.compile(r'\(\s*polygon\s+' +\
	rf'(?P<polyType>{__reId})\s+' +\
	rf'(?P<polyName>{__reId})\s+' +\
	rf'(?P<polyVtcs>{__reFloat}\s+{__reFloat}(\s+{__reFloat}\s+{__reFloat}){{2,}})' +\
	rf'\s*\)'
)



def fromFile(filepath: str) -> WorldMap:
	'''
	Creates a WorldMap object from a wrl file
	'''
	wmap = WorldMap()
	with open(filepath, 'r') as f:
		line = f.readline()
		while line:
			__parseLine(line, wmap)
			line = f.readline()
	return wmap
#end def


def __parseLine(line: str, wmap: WorldMap):
	match = __rxDimensions.match(line)
	if match:
		__parseDimensions(wmap, match)
		return
	match = __rxPolygon.match(line)
	if match:
		__parsePolygon(wmap, match)
		return
#end def


def __parseDimensions(wmap, match):
	wmap._name   = match.group('roomName')
	wmap._width  = float(match.group('roomWidth'))
	wmap._height = float(match.group('roomHeight'))
#end def


def __parsePolygon(wmap, match):
	pname = match.group('polyName')
	ptype = match.group('polyType')
	svtcs = re.split(r'\s+', match.group('polyVtcs'))
	if len(svtcs) % 2 != 0:
		return
	vtcs = []
	for i in range(0, len(svtcs), 2):
		vtcs.append( (
			float(svtcs[i]),
			float(svtcs[i+1])
		))
	p = Polygon(pname, ptype, vtcs)
	wmap._poly.append(p)
#end def


def main():
	m = fromFile('/home/dragon/Develop/catkin_ws-mrs/src/simulator/src/data/Lab/Lab.wrl')
	print(m)

if __name__ == '__main__':
	main()
