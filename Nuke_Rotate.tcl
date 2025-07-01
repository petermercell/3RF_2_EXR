set cut_paste_input [stack 0]
version 16.0 v4
push $cut_paste_input
Group {
 name Nuke_Rotate
 selected true
 xpos -57
 ypos 86
 addUserKnob {20 User l Nuke_Rotate}
 addUserKnob {41 rotate T Transform1.rotate}
}
 Input {
  inputs 0
  name Input1
  xpos 72
  ypos -17
 }
 Transform {
  rotate -90
  center {{input.width/2} {input.height/2}}
  shutteroffset centred
  name Transform1
  xpos 72
  ypos 23
 }
 Group {
  name dCrop1
  help "@b; dCrop v2@n; for Nuke 6.0v3\n\nProvides options to crop a input by box, percentage, bbox or a aspect ratio relative to the input.\n\nCreated by Diogo Girondi\ndiogogirondi@gmail.com"
  knobChanged "n = nuke.thisNode()\nk = nuke.thisKnob()\n\nif k.name() in ('mode', 'showPanel'):\n    mode = n\['mode'].value()\n    if mode == 'Box':\n        n\['cbox'].setVisible(True)\n        n\['left'].setVisible(False)\n        n\['right'].setVisible(False)\n        n\['bottom'].setVisible(False)\n        n\['top'].setVisible(False)\n        n\['cbbox'].setVisible(False)\n        n\['ar_selection'].setVisible(False)\n        n\['ar_custom'].setVisible(False)\n        n\['offset'].setVisible(False)\n    elif mode == 'BBox':\n        n\['cbox'].setVisible(False)\n        n\['left'].setVisible(False)\n        n\['right'].setVisible(False)\n        n\['bottom'].setVisible(False)\n        n\['top'].setVisible(False)\n        n\['cbbox'].setVisible(True)\n        n\['ar_selection'].setVisible(False)\n        n\['ar_custom'].setVisible(False)\n        n\['offset'].setVisible(False)\n    elif mode == 'Percentage':\n        n\['cbox'].setVisible(False)\n        n\['left'].setVisible(True)\n        n\['right'].setVisible(True)\n        n\['bottom'].setVisible(True)\n        n\['top'].setVisible(True)\n        n\['cbbox'].setVisible(False)\n        n\['ar_selection'].setVisible(False)\n        n\['ar_custom'].setVisible(False)\n        n\['offset'].setVisible(False)\n    elif mode == 'Aspect Ratio':\n        n\['cbox'].setVisible(False)\n        n\['left'].setVisible(False)\n        n\['right'].setVisible(False)\n        n\['bottom'].setVisible(False)\n        n\['top'].setVisible(False)\n        n\['cbbox'].setVisible(False)\n        n\['ar_selection'].setVisible(True)\n        if n\['ar_selection'].value() == 'Custom':\n            n\['ar_custom'].setVisible(True)\n        else:\n            n\['ar_custom'].setVisible(False)\n        if n\['ar_selection'].value() == 'Input       ':\n            n\['offset'].setEnabled(False)\n        else:\n            n\['offset'].setEnabled(True)\n        n\['offset'].setVisible(True)\n\nif k.name() in ('ar_selection', 'showPanel'):\n    aspect = n\['ar_selection'].value()\n    if aspect == 'Custom':\n        n\['ar_custom'].setVisible(True)\n    else:\n        n\['ar_custom'].setVisible(False)\n    if aspect == 'Input       ':\n        n\['offset'].setEnabled(False)\n    else:\n        n\['offset'].setEnabled(True)\n\nif k.name() == 'reformat':\n    reformat = n\['reformat'].value()\n    if reformat == True:\n        n\['crop'].setValue(False)\n    else:\n        n\['crop'].setValue(True)"
  tile_color 0xa57aaaff
  xpos 75
  ypos 88
  addUserKnob {20 dcrop l "dCrop v2"}
  addUserKnob {4 mode l Mode M {Box BBox Percentage "Aspect Ratio" ""}}
  mode BBox
  addUserKnob {26 ""}
  addUserKnob {15 cbox l Box +HIDDEN}
  cbox {0 0 2048 1152}
  addUserKnob {26 cbbox l "" +STARTLINE T "Crop area based on the input BBox."}
  addUserKnob {7 left l L +HIDDEN R 0 100}
  addUserKnob {7 right l R +HIDDEN R 0 100}
  addUserKnob {7 bottom l B +HIDDEN R 0 100}
  addUserKnob {7 top l T +HIDDEN R 0 100}
  addUserKnob {4 ar_selection l Aspect +HIDDEN M {"Input       " 1.00 1.19 1.25 1.33 1.35 1.37 1.43 1.50 1.56 1.60 1.66 1.75 1.778 1.85 2.00 2.20 2.35 2.40 2.55 2.59 2.76 4.00 Custom ""}}
  addUserKnob {7 ar_custom l " " -STARTLINE +HIDDEN R 0 4}
  ar_custom 1
  addUserKnob {7 offset l Offset +DISABLED +HIDDEN R -100 100}
  addUserKnob {26 ""}
  addUserKnob {14 softness l Softness R 0 100}
  addUserKnob {1 output l INVISIBLE +INVISIBLE}
  output "\[value this.mode]"
  addUserKnob {6 reformat +STARTLINE}
  reformat true
  addUserKnob {6 intersect -STARTLINE}
  addUserKnob {6 crop l "black outside" -STARTLINE}
 }
  Input {
   inputs 0
   name Input
   xpos 81
   ypos 139
  }
set N9bdb4c00 [stack 0]
  NoOp {
   name Aspect
   tile_color 0xff005fff
   gl_color 0xff005fff
   label Math
   xpos 240
   ypos 199
   addUserKnob {20 ar_math l "Aspect Ratio Math"}
   addUserKnob {7 input_ar R 0 4}
   input_ar {{(input.width*input.pixel_aspect)/input.height i}}
   addUserKnob {7 custom_ar R 0 4}
   custom_ar {{"parent.ar_selection==0?this.input_ar:parent.ar_selection==23?parent.ar_custom:\[value ar_selection]" i}}
   addUserKnob {6 ar_isBigger +STARTLINE}
   ar_isBigger {{this.input_ar>=this.custom_ar i}}
   addUserKnob {26 ""}
   addUserKnob {12 ar_area}
   ar_area {{ar_isBigger?((input.height*this.custom_ar)/input.pixel_aspect):input.width i} {ar_isBigger?input.height:((input.width*input.pixel_aspect)/this.custom_ar) i}}
   addUserKnob {12 ar_pos}
   ar_pos {{((input.width-this.ar_area.x)/2)*(parent.offset/100) i} {((input.height-this.ar_area.y)/2)*(parent.offset/100) i}}
   addUserKnob {15 ar}
   ar {{ar_isBigger?this.ar_pos.x+((input.width/2)-(((input.height*this.custom_ar)/input.pixel_aspect)/2)):0 i} {ar_isBigger?0:this.ar_pos.y+((input.height/2)-(((input.width*input.pixel_aspect)/this.custom_ar)/2)) i} {ar_isBigger?this.ar_pos.x+((input.width/2)+(((input.height*this.custom_ar)/input.pixel_aspect)/2)):input.width i} {ar_isBigger?input.height:this.ar_pos.y+((input.height/2)+(((input.width*input.pixel_aspect)/this.custom_ar)/2)) i}}
  }
  Crop {
   box {{parent.Aspect.ar.main i} {parent.Aspect.ar.main i} {parent.Aspect.ar.main i} {parent.Aspect.ar.main i}}
   softness {{parent.softness.w i} {parent.softness.h i}}
   reformat {{parent.reformat i}}
   intersect {{parent.intersect i}}
   crop {{parent.crop i}}
   name AspectRatio
   label Crop
   xpos 240
   ypos 244
  }
push $N9bdb4c00
  Crop {
   box {{"width * ( parent.left / 100 )" i} {"height * ( parent.bottom / 100 )" i} {"width - ( width * ( parent.right / 100 ) )" i} {"height - ( height * ( parent.top / 100 ) )" i}}
   softness {{parent.softness.w i} {parent.softness.h i}}
   reformat {{parent.reformat i}}
   intersect {{parent.intersect i}}
   crop {{parent.crop i}}
   name Percentage
   label Crop
   xpos 125
   ypos 246
  }
push $N9bdb4c00
  Crop {
   box {{input.bbox.x i} {input.bbox.y i} {input.bbox.r i} {input.bbox.t i}}
   softness {{parent.softness.w i} {parent.softness.h i}}
   reformat {{parent.reformat i}}
   intersect {{parent.intersect i}}
   crop {{parent.crop i}}
   name BBox
   label Crop
   xpos 15
   ypos 246
  }
push $N9bdb4c00
  Crop {
   box {{parent.cbox.x i} {parent.cbox.y i} {parent.cbox.r i} {parent.cbox.t i}}
   softness {{parent.softness.w i} {parent.softness.h i}}
   reformat {{parent.reformat i}}
   intersect {{parent.intersect i}}
   crop {{parent.crop i}}
   name Box
   label Crop
   xpos -95
   ypos 246
  }
  Switch {
   inputs 4
   which {{parent.mode i}}
   name Mode
   xpos 80
   ypos 349
  }
  Output {
   name Output1
   xpos 80
   ypos 425
  }
  NoOp {
   inputs 0
   name Aspect1
   tile_color 0xff005fff
   gl_color 0xff005fff
   label Math
   xpos 637
   ypos 228
   addUserKnob {20 ar_math l "Aspect Ratio Math"}
   addUserKnob {7 input_ar R 0 4}
   input_ar {{(input.width*input.pixel_aspect)/input.height i}}
   addUserKnob {7 custom_ar R 0 4}
   custom_ar {{"parent.ar_selection==0?this.input_ar:parent.ar_selection==23?parent.ar_custom:\[value ar_selection]" i}}
   addUserKnob {6 ar_isBigger +STARTLINE}
   ar_isBigger {{this.input_ar>this.custom_ar i}}
   addUserKnob {26 ""}
   addUserKnob {12 teste}
   addUserKnob {12 ar_pos}
   ar_pos {{this.ar_isBigger?parent.offset*(width/100):0 i} {this.ar_isBigger?0:parent.offset*(width/100) i}}
   addUserKnob {15 ar}
   ar {{"this.custom_ar > this.input_ar ? 0 + ar_pos.x :  ( ( width / 2 ) - ( ( ( height * custom_ar ) / pixel_aspect ) / 2 ) ) + ar_pos.x" i} {"this.custom_ar < this.input_ar ? 0 + ar_pos.y : ( ( height / 2 ) - ( ( ( width * pixel_aspect ) / custom_ar ) / 2 ) ) + ar_pos.y" i} {"this.custom_ar > this.input_ar ? width + ar_pos.x : ( ( width / 2 ) + ( ( ( height * custom_ar ) / pixel_aspect ) / 2 ) ) + ar_pos.x" i} {"this.custom_ar < this.input_ar ? height + ar_pos.y : ( ( height / 2 ) + ( ( ( width * pixel_aspect ) / custom_ar ) / 2 ) ) + ar_pos.y" i}}
  }
 end_group
 Output {
  name Output1
  xpos 75
  ypos 188
 }
end_group
