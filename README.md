# k_react
K_react is a modal operator useful for creating retopology geometry in Blender. The operator once activated is trying to create new faces or triangles using the preexisting geometry. By default k_react creates a new snapped vertex that is used to create a quad with the existing geometry. You can change the position of the vertex just by hovering with the mouse. Once you get the desired quad hit Left Mouse to place the vertex. For previewing and creating a triagle just hold the Shift key and for previewing/creating a line hold Ctrl+Shift. If by hovering the mouse you get close to an preexisting vertex the preview vertex merges with the geometry, in this way you can fill holes for example. K_react is contantly searching near-by geometry to make appropriate quads or trigs but if you want just to hold on the current previewed face (and avoid previewing a new one or the auto merge behaviour) press X. For undo hold the mouse steady and press Z. You can exit the modal operator by hitting the Right Mouse. In typical Blender fashion, you can also check the modal operator hotkey in the header when using it.

![](/img/quick_tut_02.gif)

# Install

Download md_operator_kreact_2_79.py for Blender 2.79 or md_operator_kreact_2_80.py for Blender 2.8x, go to Blender User Preferences (Ctrl Alt U), choose Add-ons tab, click Install Add-on from File, select md_operator_kreact_2_79.py (or md_operator_kreact_2_80.py). Finally click the checkbox to activate the add-on.

Personally I'm using Ctrl + RightClick hotkey. You can set a hotkey in the Hotkey (Edit>Preferences) tab by adding a new hotkey in the 3D View > Mesh > Mesh (Global) section. The Blender command for the operator is md.k_react .

![](img/hotkey.png)

