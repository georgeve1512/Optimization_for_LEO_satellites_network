#!/usr/bin/env python
from xml.dom import minidom
import os

speed = 50
angle = 90

# Open the file that has grid coordinates
with open("coordinates.txt") as f:
    lines = f.read().splitlines()

# Parse coordinates
# Format is 'Satellite i pos: (posX, posY)'
posX = []
posY = []
for line in lines:
    posX.append(float(line[line.find("(",0,len(line))+1:line.find(", ",0,len(line))]))
    posY.append(float(line[line.find(", ",0,len(line))+1:line.find(")",0,len(line))]))

# Create XML file
root = minidom.Document()

# Create highest level - "movements"
# <movements> ...
xmlHigherLevel = root.createElement('movements')
root.appendChild(xmlHigherLevel)

xml_str = ""    # Result string, later will be file's content
save_path_file = "mobility.xml"  # Output file name

for i in range(0, len(posX)):
    # Create a new movement with id='i'
    # <movement id="">...
    xml = root.createElement('movement')
    xml.setAttribute('id',str(i))
    xmlHigherLevel.appendChild(xml) # <movements> <movement id="[i]">

    # Create attributes for TurtleMobility
    # <set x="" y="" ...>
    child = root.createElement('set')
    child.setAttribute('x',str(posX[i]))
    child.setAttribute('y', str(posY[i]))
    child.setAttribute('speed', str(speed))
    child.setAttribute('angle', str(angle))
    child.setAttribute('borderPolicy',"wrap")
    xml.appendChild(child) # <movements> <movement id="[i]"> <set ...>

    # Create repeat section
    repeat = root.createElement('repeat')
    repeatChild = root.createElement('forward')
    repeatChild.setAttribute('t', "60")
    repeat.appendChild(repeatChild)
    xml.appendChild(repeat) #<movements> <movement id="[i]"> <repeat> <forward ...>

    # Update result string
    xml_str = root.toprettyxml(indent ="\t")
    
with open(save_path_file, "w") as f:
    f.write(xml_str)

for i in range(0, len(posX)):
    print("*.rte[" + str(i) + "].mobility.turtleScript = xmldoc(\"mobility.xml\", \"movements//movement[@id='" + str(i) + "']\")")
print("Done!")


