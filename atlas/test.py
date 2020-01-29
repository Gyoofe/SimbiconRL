from pyassimp import *

scene = load('ltorso.stl')
#print(scene.rootnode.children)
#print(scene)
#print(scene.rootnode)
for c in scene.rootnode.children:
    print(str(c))

