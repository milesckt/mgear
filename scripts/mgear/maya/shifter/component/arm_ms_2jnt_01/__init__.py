"""Component Arm Ms 2 joints 01 module"""

# ms 2jnt feature -----------------------
# //done//FK isolation
# //done//FK roll ctl
# //done//Independent IK-FK hand ctl
# //done//IK auto up vector(default off)
# //done//T pose centric FK ctl
# //done//elbow thickness + seperate upper/lower limb roundness ctl
# //done//elbow scl and hand scl(ik/fk) add to jt scl
# //done//addition limb jt layer ctl(optional)
# To Do List -------------------------------

# upper sleeve lower sleeve ctl(optional)


'''Upgraded feature
removed nodes, increased speed
replace roundness with free tangent ctl on mgear roll solver

'''
import pymel.core as pm
from pymel.core import datatypes

from mgear.maya.shifter import component

from mgear.maya import node, fcurve, applyop, vector, icon
from mgear.maya import attribute, transform, primitive

#############################################
# COMPONENT
#############################################


class Component(component.Main):
    """Shifter component Class"""

    # =====================================================
    # OBJECTS
    # =====================================================
    def addObjects(self):
        """Add all the objects needed to create the component."""

        self.normal = self.getNormalFromPos(self.guide.apos)
        self.binormal = self.getBiNormalFromPos(self.guide.apos)

        self.length0 = vector.getDistance(
            self.guide.apos[0], self.guide.apos[1])
        self.length1 = vector.getDistance(
            self.guide.apos[1], self.guide.apos[2])
        self.length2 = vector.getDistance(
            self.guide.apos[2], self.guide.apos[3])

        # FK Controlers -----------------------------------
        # *ms* set npo @ Tpose, to make the fk rotation work
        # best with rot order"yzx"

        self.fk_cns = primitive.addTransformFromPos(
            self.root, self.getName("fk_cns"), self.guide.apos[0])

        vec_offset = ((self.guide.apos[1] - self.guide.apos[0]) * [1, 0, 0])
        tpv = self.guide.apos[0] + vec_offset

        t = transform.getTransformLookingAt(
            self.guide.apos[0], tpv, self.normal, "xz", self.negate)
        # *ms* add FK isolation
        self.fk0_npo = primitive.addTransform(
            self.fk_cns, self.getName("fk0_npo"), t)

        t = transform.getTransformLookingAt(self.guide.apos[0],
                                            self.guide.apos[1],
                                            self.normal, "xz",
                                            self.negate)

        po_off = datatypes.Vector(.35 * self.length0 * self.n_factor, 0, 0)

        self.fk0_ctl = self.addCtl(self.fk0_npo,
                                   "fk0_ctl",
                                   t,
                                   self.color_fk,
                                   "cube",
                                   w=self.length0 * .7,
                                   h=self.size * .1,
                                   d=self.size * .1,
                                   po=po_off,
                                   tp= self.parentCtlTag)
        attribute.setKeyableAttributes(self.fk0_ctl)
        # *ms* add fk roll control Simage style
        po_off = datatypes.Vector(.85 * self.length0 * self.n_factor, 0, 0)

        self.fk0_roll_ctl = self.addCtl(self.fk0_ctl,
                                        "fk0_roll_ctl",
                                        t, self.color_fk,
                                        "cube", w=self.length0 * .3,
                                        h=self.size * .1,
                                        d=self.size * 0.1,
                                        po=po_off,
                                        tp=self.fk0_ctl)

        attribute.setRotOrder(self.fk0_roll_ctl, "YZX")
        attribute.setKeyableAttributes(self.fk0_roll_ctl, ["rx"])
        self.fk0_mtx = primitive.addTransform(
            self.fk0_roll_ctl, self.getName("fk0_mtx"), t)

        t = transform.setMatrixPosition(t, self.guide.apos[1])

        self.fk1_ref = primitive.addTransform(
            self.fk0_roll_ctl, self.getName("fk1_ref"), t)

        self.fk1_loc = primitive.addTransform(self.root, self.getName("fk1_loc"), t)

        t = transform.getTransformLookingAt(self.guide.apos[1],
                                            self.guide.apos[2],
                                            self.normal,
                                            "xz",
                                            self.negate)

        self.fk1_npo = primitive.addTransform(
            self.fk1_loc, self.getName("fk1_npo"), t)

        po_off = datatypes.Vector(.35 * self.length1 * self.n_factor, 0, 0)
        self.fk1_ctl = self.addCtl(self.fk1_npo,
                                   "fk1_ctl",
                                   t,
                                   self.color_fk,
                                   "cube",
                                   w=self.length1 * .7,
                                   h=self.size * .1,
                                   d=self.size * .1,
                                   po=po_off, tp=self.fk0_roll_ctl)

        attribute.setKeyableAttributes(self.fk1_ctl)

        self.fk1_mtx = primitive.addTransform(
            self.fk1_ctl, self.getName("fk1_mtx"), t)

        po_off = datatypes.Vector(.85 * self.length1 * self.n_factor, 0, 0)
        self.fk1_roll_ctl = self.addCtl(self.fk1_ctl,
                                        "fk1_roll_ctl",
                                        t,
                                        self.color_fk,
                                        "cube",
                                        w=self.length1 * .3,
                                        h=self.size * .1,
                                        d=self.size * .1,
                                        po=po_off, tp=self.fk1_ctl)
        attribute.setRotOrder(self.fk1_roll_ctl, "XYZ")
        attribute.setKeyableAttributes(self.fk1_roll_ctl, ["rx"])

        t = transform.getTransformLookingAt(self.guide.apos[2],
                                            self.guide.apos[3],
                                            self.normal,
                                            "xz",
                                            self.negate)
        # *ms* buffer object to feed into ikfk solver for hand seperation
        self.fk2_mtx = primitive.addTransform(self.fk1_roll_ctl,
                                              self.getName("fk2_mtx"),
                                              t)

        # fk2_loc is need to take the effector position + bone1 rotation
        t1 = transform.getTransformLookingAt(self.guide.apos[2],
                                             self.guide.apos[1],
                                             self.normal,
                                             "-xz",
                                             self.negate)

        self.fk2_loc = primitive.addTransform(
            self.root, self.getName("fk2_loc"), t1)

        self.fk2_npo = primitive.addTransform(self.fk2_loc,
                                              self.getName("fk2_npo"),
                                              t)
        po_off = datatypes.Vector(.5 * self.length2 * self.n_factor, 0, 0)
        self.fk2_ctl = self.addCtl(self.fk2_npo,
                                   "fk2_ctl",
                                   t,
                                   self.color_fk,
                                   "cube",
                                   w=self.length2,
                                   h=self.size * .1,
                                   d=self.size * .1,
                                   po=po_off,
                                   tp=self.fk1_roll_ctl)
        attribute.setKeyableAttributes(self.fk2_ctl)

        self.fk_ctl = [self.fk0_roll_ctl, self.fk1_mtx, self.fk2_ctl]
        self.fk_ctls = [self.fk0_ctl,
                        self.fk0_roll_ctl,
                        self.fk1_ctl,
                        self.fk1_roll_ctl,
                        self.fk2_ctl]

        for x in self.fk_ctls:
            attribute.setInvertMirror(x, ["tx", "ty", "tz"])

        # IK Controlers -----------------------------------

        self.ik_cns = primitive.addTransformFromPos(
            self.root, self.getName("ik_cns"), self.guide.pos["wrist"])

        self.ikcns_ctl = self.addCtl(
            self.ik_cns,
            "ikcns_ctl",
            transform.getTransformFromPos(self.guide.pos["wrist"]),
            self.color_ik,
            "null",
            w=self.size * .12, tp=self.parentCtlTag)
        attribute.setInvertMirror(self.ikcns_ctl, ["tx", "ry", "rz"])

        if self.negate:
            m = transform.getTransformLookingAt(self.guide.pos["wrist"],
                                                self.guide.pos["eff"],
                                                self.normal,
                                                "x-y",
                                                True)
        else:
            m = transform.getTransformLookingAt(self.guide.pos["wrist"],
                                                self.guide.pos["eff"],
                                                self.normal,
                                                "xy",
                                                False)
        self.ik_ctl = self.addCtl(self.ikcns_ctl,
                                  "ik_ctl",
                                  m,
                                  self.color_ik,
                                  "cube",
                                  w=self.size * .12,
                                  h=self.size * .12,
                                  d=self.size * .12,
                                  tp=self.ikcns_ctl)
        attribute.setKeyableAttributes(self.ik_ctl)
        attribute.setInvertMirror(self.ik_ctl, ["tx", "ry", "rz"])
        #empty transfor for synoptic view ikfk script to run
        self.ikRot_ctl = primitive.addTransformFromPos(self.ik_ctl, self.getName("ikRot_ctl"), self.guide.pos["wrist"])


        # upv
        v = self.guide.apos[2] - self.guide.apos[0]
        v = self.normal ^ v
        v.normalize()
        v *= self.size * .5
        v += self.guide.apos[1]
        # *ms* auto up vector ------------------------------
        self.upv_org = primitive.addTransformFromPos(self.root,
                                                     self.getName("upv_org"),
                                                     self.guide.apos[0])
        self.upv_cns = primitive.addTransformFromPos(self.root,
                                                     self.getName("upv_cns"),
                                                     self.guide.apos[0])
        self.upv_auv = primitive.addTransformFromPos(self.upv_org,
                                                     self.getName("upv_auv"),
                                                     self.guide.apos[0])
        self.upv_mtx = primitive.addTransformFromPos(self.upv_cns,
                                                     self.getName("upv_mtx"),
                                                     self.guide.apos[0])

        self.upv_npo = primitive.addTransformFromPos(self.upv_mtx,
                                                     self.getName("upv_npo"),
                                                     v)
        self.upv_ctl = self.addCtl(self.upv_npo,
                                   "upv_ctl",
                                   transform.getTransform(self.upv_npo),
                                   self.color_ik,
                                   "diamond",
                                   w=self.size * .12,
                                   tp=self.parentCtlTag)
        attribute.setKeyableAttributes(self.upv_ctl, self.t_params)
        if self.negate:
                self.upv_npo.rz.set(180)
                self.upv_npo.sy.set(-1)
        # attribute.setInvertMirror(self.upv_ctl, ["tx"])

        # References --------------------------------------
        # Calculate  again the transfor for the IK ref. This way align with FK
        trnIK_ref = transform.getTransformLookingAt(self.guide.pos["wrist"],
                                                    self.guide.pos["eff"],
                                                    self.normal,
                                                    "xz",
                                                    self.negate)
        self.ik_ref = primitive.addTransform(self.ik_ctl,
                                             self.getName("ik_ref"),
                                             trnIK_ref)
        self.fk_ref = primitive.addTransform(self.fk_ctl[2],
                                             self.getName("fk_ref"),
                                             trnIK_ref)

        # Chain --------------------------------------------
        # take outputs of the ikfk2bone solver
        self.bone0 = primitive.addLocator(
            self.root,
            self.getName("0_bone"),
            transform.getTransform(self.fk_ctl[0]))

        self.bone0_shp = self.bone0.getShape()
        self.bone0_shp.setAttr("localPositionX", self.n_factor * .5)
        self.bone0_shp.setAttr("localScale", .5, 0, 0)
        self.bone0.setAttr("sx", self.length0)
        self.bone0.setAttr("visibility", False)

        self.bone1 = primitive.addLocator(
            self.root,
            self.getName("1_bone"),
            transform.getTransform(self.fk_ctl[1]))

        self.bone1_shp = self.bone1.getShape()
        self.bone1_shp.setAttr("localPositionX", self.n_factor * .5)
        self.bone1_shp.setAttr("localScale", .5, 0, 0)
        self.bone1.setAttr("sx", self.length1)
        self.bone1.setAttr("visibility", False)

        self.ctrn_loc = primitive.addTransformFromPos(self.root,
                                                      self.getName("ctrn_loc"),
                                                      self.guide.apos[1])
        # eff npo --- take the effector output of gear ik solver
        # eff loc --- take the fk ik blend result
        self.eff_loc = primitive.addTransformFromPos(self.root,
                                                     self.getName("eff_loc"),
                                                     self.guide.apos[2])

        # Mid Controler ------------------------------------
        t = transform.getTransform(self.ctrn_loc)
        self.mid_cns = primitive.addTransform(self.ctrn_loc,
                                                      self.getName("mid_cns"),
                                                      t)

        self.mid_ctl = self.addCtl(self.mid_cns,
                                   "mid_ctl",
                                   transform.getTransform(self.ctrn_loc),
                                   self.color_ik,
                                   "sphere",
                                   w=self.size * .2,
                                   tp=self.parentCtlTag)
        # attribute.setInvertMirror(self.mid_ctl, ["tx", "ty", "tz"])
        if self.negate:
                self.mid_cns.rz.set(180)
                self.mid_cns.sz.set(-1)

        self.mid_loc = primitive.addTransform(
            self.mid_cns,
            self.getName("mid_loc"),
            t)
        # *ms* add elbow thickness

        # Roll join ref

        self.tws0_npo = primitive.addTransform(
            self.root,
            self.getName("tws0_npo"),
            transform.getTransform(self.fk_ctl[0]))
        self.tws0_loc = primitive.addTransform(
            self.tws0_npo,
            self.getName("tws0_loc"),
            transform.getTransform(self.fk_ctl[0]))
        self.tws0_roll = primitive.addTransform(
            self.tws0_loc,
            self.getName("tws0_roll"),
            transform.getTransform(self.fk_ctl[0]))      
        self.tws0_rot = primitive.addTransform(
            self.tws0_roll,
            self.getName("tws0_rot"),
            transform.getTransform(self.fk_ctl[0]))

        # self.tws1_npo = primitive.addTransform(
        #     self.ctrn_loc,
        #     self.getName("tws1_npo"),
        #     transform.getTransform(self.ctrn_loc))
        self.tws1_loc = primitive.addTransform(
            self.mid_loc,
            self.getName("tws1_loc"),
            transform.getTransform(self.ctrn_loc))
        self.tws1_rot = primitive.addTransform(
            self.tws1_loc,
            self.getName("tws1_rot"),
            transform.getTransform(self.ctrn_loc))

        self.tws2_loc = primitive.addTransform(
            self.mid_loc,
            self.getName("tws2_loc"),
            transform.getTransform(self.ctrn_loc))
        self.tws2_rot = primitive.addTransform(
            self.tws2_loc,
            self.getName("tws2_rot"),
            transform.getTransform(self.ctrn_loc))

        self.tws3_npo = primitive.addTransform(
            self.root,
            self.getName("tws3_npo"),
            transform.getTransform(self.fk_ctl[2]))
        self.tws3_loc = primitive.addTransform(
            self.tws3_npo,
            self.getName("tws3_loc"),
            transform.getTransform(self.fk_ctl[2]))
        self.tws3_rot = primitive.addTransform(
            self.tws3_loc,
            self.getName("tws3_rot"),
            transform.getTransform(self.fk_ctl[2]))

        ## Add tangent control

        self.tan0_npo = primitive.addTransform(
            self.tws0_loc,
            self.getName("tan0_npo"),
            transform.getTransform(self.tws0_rot))
        self.tan0_npo.tx.set(self.length0*0.3)

        self.tan1_npo = primitive.addTransform(
            self.tws1_loc,
            self.getName("tan1_npo"),
            transform.getTransform(self.tws1_rot))
        self.tan1_npo.tx.set(self.length0*-1/3)
        self.tan2_npo = primitive.addTransform(
            self.tws2_loc,
            self.getName("tan2_npo"),
            transform.getTransform(self.tws2_rot))
        self.tan2_npo.tx.set(self.length1 /3)
        self.tan3_npo = primitive.addTransform(
            self.tws3_loc,
            self.getName("tan3_npo"),
            transform.getTransform(self.tws3_rot))
        self.tan3_npo.tx.set(self.length1* -1/3)
        po_off = datatypes.Vector(0,.3 * self.length0 * self.n_factor, 0)
        self.tan_ctls = []

        self.tan0_ctl = self.addCtl(self.tan0_npo,
                                   "upTan0_ctl",
                                   transform.getTransform(self.tan0_npo),
                                   self.color_ik,
                                   "sphere",
                                   w=self.size * .03,
                                   h=self.size * .03,
                                   d=self.size * .03,
                                   po=po_off,
                                   tp=self.mid_ctl)
        self.tan_ctls.append(self.tan0_ctl)

        self.tan1_ctl = self.addCtl(self.tan1_npo,
                                   "upTan1_ctl",
                                   transform.getTransform(self.tan1_npo),
                                   self.color_ik,
                                   "sphere",
                                   w=self.size * .03,
                                   h=self.size * .03,
                                   d=self.size * .03,
                                   po=po_off,
                                   tp=self.mid_ctl)
        self.tan_ctls.append(self.tan1_ctl)

        po_off = datatypes.Vector(0,.3 * self.length1 * self.n_factor, 0)

        self.tan2_ctl = self.addCtl(self.tan2_npo,
                                   "dnTan0_ctl",
                                   transform.getTransform(self.tan2_npo),
                                   self.color_ik,
                                   "sphere",
                                   w=self.size * .03,
                                   h=self.size * .03,
                                   d=self.size * .03,
                                   po=po_off,
                                   tp=self.mid_ctl)
        self.tan_ctls.append(self.tan2_ctl)

        self.tan3_ctl = self.addCtl(self.tan3_npo,
                                   "dnTan1_ctl",
                                   transform.getTransform(self.tan3_npo),
                                   self.color_ik,
                                   "sphere",
                                   w=self.size * .03,
                                   h=self.size * .03,
                                   d=self.size * .053,
                                   po= po_off,
                                   tp=self.mid_ctl)
        self.tan_ctls.append(self.tan3_ctl)

        for tan in self.tan_ctls:
          attribute.setKeyableAttributes(tan,["tx","ty","tz"])
        # Divisions ----------------------------------------
        # We have at least one division at the start, the end and one for the
        # elbow. + 2 for elbow angle control
        # separate up and dn limb
        self.divisions = self.settings["div0"] + self.settings["div1"] + 3 + 2
        self.divisions0 = self.settings["div0"] + 2
        self.divisions1 = self.settings["div1"] + 2

        self.div_cns = []
        self.div_cnsUp = []
        self.div_cnsDn = []
        self.div_ctls = []

        self.div_org = primitive.addTransform(
            self.root,
            self.getName("div_org"),
            transform.getTransform(self.root))
        self.previousTag = self.parentCtlTag
        for i in range(self.divisions0):

            div_cns = primitive.addTransform(
                self.div_org, self.getName("div%s_loc" % i))

            if self.negate:
                div_ctl = self.addCtl(
                    div_cns,
                    self.getName("div%s_ctl" % i),
                    transform.getTransform(div_cns),
                    self.color_fk, "square", d=self.size * .05,
                    w=self.size * .1,
                    po=datatypes.Vector(0, self.size * -0.05, 0),
                    ro=datatypes.Vector(0, 0, datatypes.radians(90)),
                    tp=self.previousTag)
            else:
                div_ctl = self.addCtl(
                    div_cns,
                    self.getName("div%s_ctl" % i),
                    transform.getTransform(div_cns),
                    self.color_fk,
                    "square",
                    d=self.size * .05,
                    w=self.size * .1,
                    po=datatypes.Vector(0, self.size * 0.05, 0),
                    ro=datatypes.Vector(0, 0, datatypes.radians(90)),
                    tp=self.previousTag)
            attribute.setKeyableAttributes(div_ctl)
            self.previousTag = div_ctl
            self.div_cns.append(div_cns)
            self.div_cnsUp.append(div_cns)
            self.jnt_pos.append([div_ctl, i])
            self.div_ctls.append(div_ctl)
        # mid division
        d = self.divisions0
        self.div_mid = primitive.addTransform(
            self.div_org,
            self.getName("div%s_loc" % d),
            transform.getTransform(self.mid_ctl))
        if self.negate:
            self.div_mid_ctl = self.addCtl(
                self.div_mid,
                self.getName("div%s_ctl" % d),
                transform.getTransform(self.div_mid),
                self.color_fk,
                "square",
                d=self.size * .05,
                w=self.size * .1,
                po=datatypes.Vector(0, self.size * -0.05, 0),
                ro=datatypes.Vector(0, 0, datatypes.radians(90)),
                tp=self.previousTag)
        else:
            self.div_mid_ctl = self.addCtl(
                self.div_mid, self.getName("div%s_ctl" % d),
                transform.getTransform(self.div_mid),
                self.color_fk,
                "square",
                d=self.size * .05,
                w=self.size * .1,
                po=datatypes.Vector(0, self.size * 0.05, 0),
                ro=datatypes.Vector(0, 0, datatypes.radians(90)),
                tp=self.previousTag)
        attribute.setKeyableAttributes(self.div_mid_ctl)
        self.previousTag = div_ctl

        self.div_cns.append(self.div_mid)
        self.jnt_pos.append([self.div_mid_ctl, self.divisions0])
        self.div_ctls.append(self.div_mid_ctl)
        # down division
        for i in range(self.divisions1):

            dd = i + self.divisions1 + 1
            div_cns = primitive.addTransform(
                self.div_org, self.getName("div%s_loc" % dd))
            if self.negate:
                div_ctl = self.addCtl(
                    div_cns,
                    self.getName("div%s_ctl" % dd),
                    transform.getTransform(div_cns),
                    self.color_fk,
                    "square",
                    d=self.size * .05,
                    w=self.size * .1,
                    po=datatypes.Vector(0, self.size * -0.05, 0),
                    ro=datatypes.Vector(0, 0, datatypes.radians(90)),
                    tp=self.previousTag)
            else:
                div_ctl = self.addCtl(
                    div_cns,
                    self.getName("div%s_ctl" % dd),
                    transform.getTransform(div_cns),
                    self.color_fk,
                    "square",
                    d=self.size * .05,
                    w=self.size * .1,
                    po=datatypes.Vector(0, self.size * 0.05, 0),
                    ro=datatypes.Vector(0, 0, datatypes.radians(90)),
                    tp=self.previousTag)
            attribute.setKeyableAttributes(div_ctl)
            self.previousTag = div_ctl

            self.div_cns.append(div_cns)
            self.div_cnsDn.append(div_cns)
            self.jnt_pos.append([div_ctl, i + self.divisions0 + 1])
            self.div_ctls.append(div_ctl)

        # End reference ------------------------------------
        # To help the deformation on the wrist
        self.jnt_pos.append([self.eff_loc, 'end'])

        # match IK FK references


        self.match_fk0 = primitive.addTransform(
            self.root,
            self.getName("fk0_mth"),
            transform.getTransform(self.fk_ctl[0]))
        
        self.match_fk1 = primitive.addTransform(
            self.root,
            self.getName("fk1_mth"),
            transform.getTransform(self.fk_ctl[1]))
        
        self.match_fk2 = primitive.addTransform(
            self.ik_ctl,
            self.getName("fk2_mth"),
            transform.getTransform(self.fk_ctl[2]))
        
        self.match_ik = primitive.addTransform(
            self.fk2_ctl,
            self.getName("ik_mth"),
            transform.getTransform(self.ik_ctl))

        self.match_ikRot = primitive.addTransform(
            self.fk2_ctl,
            self.getName("ikRot_mth"),
            transform.getTransform(self.ik_ctl))

        self.match_ikUpv = primitive.addTransform(
            self.fk0_roll_ctl,
            self.getName("upv_mth"),
            transform.getTransform(self.upv_ctl))



        # add visual reference
        self.line_ref = icon.connection_display_curve(
            self.getName("visalRef"), [self.upv_ctl, self.mid_ctl])

    # =====================================================
    # ATTRIBUTES
    # =====================================================
    def addAttributes(self):
        """Create the anim and setupr rig attributes for the component"""

        # Anim -------------------------------------------
        self.blend_att = self.addAnimParam(
            "blend", "Fk/Ik Arm", "double", self.settings["blend"], 0, 1)
        self.blend2_att = self.addAnimParam(
            "blend_hand", "Fk/Ik Hand", "double", self.settings["blend"], 0, 1)
        self.auv_att = self.addAnimParam(
            "auv", "Auto Upvector", "double", 0, 0, 1)
        self.roll_att = self.addAnimParam(
            "roll", "Roll", "double", 0, -180, 180)
        self.armpit_roll_att = self.addAnimParam(
            "aproll", "Armpit Roll", "double", 0, -360, 360)
        self.scale_att = self.addAnimParam(
            "ikscale", "Scale", "double", 1, .001, 99)
        self.maxstretch_att = self.addAnimParam("maxstretch",
                                                "Max Stretch",
                                                "double",
                                                self.settings["maxstretch"],
                                                1,
                                                99)
        self.slide_att = self.addAnimParam(
            "slide", "Slide", "double", .5, 0, 1)
        self.softness_att = self.addAnimParam(
            "softness", "Softness", "double", 0, 0, 1)
        self.reverse_att = self.addAnimParam(
            "reverse", "Reverse", "double", 0, 0, 1)
        self.roundness0_att = self.addAnimParam(
            "roundness_up", "Roundness Up", "double", 0, 0,1, self.size)
        self.roundness1_att = self.addAnimParam(
            "roundness_dn", "Roundness Dn", "double", 0, 0,1, self.size)
        self.volume_att = self.addAnimParam(
            "volume", "Volume", "double", 1, 0, 1)
        self.elbow_thickness_att = self.addAnimParam("elbowthickness",
                                                     "Elbow Thickness",
                                                     "double",
                                                     self.settings["elbow"],
                                                     0,
                                                     5)
        self.jntctl_vis_att = self.addAnimParam(
            "jntct_vis", "Joint Ctl Vis", "bool", 0)
        self.tan_vis_att = self.addAnimParam(
            "tan_vis", "Tangent Ctl Vis", "bool", 0)
        # Ref
        if self.settings["fkrefarray"]:
            ref_names = self.settings["fkrefarray"].split(",")
            if len(ref_names) > 1:
                self.ref_att = self.addAnimEnumParam(
                    "fkref",
                    "Fk Ref",
                    0,
                    self.settings["fkrefarray"].split(","))
                attribute.addProxyAttribute(self.ref_att,
                                            [self.fk0_ctl,
                                             self.fk1_ctl])

        if self.settings["ikrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["ikrefarray"].split(","))
            if len(ref_names) > 1:
                self.ikref_att = self.addAnimEnumParam(
                    "ikref",
                    "Ik Ref",
                    0,
                    ref_names)
                attribute.addProxyAttribute(self.ikref_att,[self.ik_ctl])

        if self.settings["upvrefarray"]:
            ref_names = self.get_valid_alias_list(
                self.settings["upvrefarray"].split(","))
            if len(ref_names) > 1:
                self.upvref_att = self.addAnimEnumParam(
                    "upvref",
                    "UpV Ref",
                    0,
                    ref_names)
                attribute.addProxyAttribute(self.upvref_att,[self.upv_ctl])
        if self.validProxyChannels:
           
            attribute.addProxyAttribute(
                self.blend_att,
                [self.fk0_ctl,
                    self.fk1_ctl,
                    self.fk2_ctl,
                    self.mid_ctl,
                    self.ik_ctl])
            attribute.addProxyAttribute(self.blend2_att,[self.fk2_ctl,self.ik_ctl])
            attribute.addProxyAttribute(self.roundness0_att,[self.mid_ctl])
            attribute.addProxyAttribute(self.roundness1_att,[self.mid_ctl])
            attribute.addProxyAttribute(self.tan_vis_att,[self.mid_ctl])
            attribute.addProxyAttribute(self.auv_att,[self.upv_ctl])
            attribute.addProxyAttribute(self.roll_att,[self.ik_ctl, self.upv_ctl])
            attribute.addProxyAttribute(self.armpit_roll_att,[self.mid_ctl])
            attribute.addProxyAttribute(self.elbow_thickness_att,[self.mid_ctl])
            attribute.addProxyAttribute(self.jntctl_vis_att,[self.mid_ctl])
        # Setup ------------------------------------------
        # Eval Fcurve
        self.st_value = fcurve.getFCurveValues(self.settings["st_profile"],
                                               self.divisions)
        self.sq_value = fcurve.getFCurveValues(self.settings["sq_profile"],
                                               self.divisions)

        self.st_att = [self.addSetupParam("stretch_%s" % i,
                                          "Stretch %s" % i,
                                          "double",
                                          self.st_value[i],
                                          -1,
                                          0)
                       for i in range(self.divisions)]

        self.sq_att = [self.addSetupParam("squash_%s" % i,
                                          "Squash %s" % i,
                                          "double",
                                          self.sq_value[i],
                                          0,
                                          1)
                       for i in range(self.divisions)]

        self.resample_att = self.addSetupParam(
            "resample", "Resample", "bool", True)
        self.absolute_att = self.addSetupParam(
            "absolute", "Absolute", "bool", False)
        
        self.keep_node = []
        self.keep_node.append(self.match_fk0)
        self.keep_node.append(self.match_fk1)
        self.keep_node.append(self.match_fk2)
        self.keep_node.append(self.match_ik)
        self.keep_node.append(self.match_ikRot)
        self.keep_node.append(self.match_ikUpv)
        self.keep_node.append(self.ikRot_ctl)
        for m in self.keep_node:
          name = "keep_node"
          pm.addAttr(m, at="message", ln= name)
          self.root.message >> m+"."+name

    # =====================================================
    # OPERATORS
    # =====================================================
    def addOperators(self):
        """Create operators and set the relations for the component rig

        Apply operators, constraints, expressions to the hierarchy.
        In order to keep the code clean and easier to debug,
        we shouldn't create any new object in this method.

        """

        # Visibilities -------------------------------------
        # fk
        fkvis_node = node.createReverseNode(self.blend_att)
        for shp in self.fk0_ctl.getShapes():
            pm.connectAttr(fkvis_node + ".outputX", shp.attr("visibility"))
        for shp in self.fk0_roll_ctl.getShapes():
            pm.connectAttr(fkvis_node + ".outputX", shp.attr("visibility"))
        for shp in self.fk1_ctl.getShapes():
            pm.connectAttr(fkvis_node + ".outputX", shp.attr("visibility"))
        for shp in self.fk1_roll_ctl.getShapes():
            pm.connectAttr(fkvis_node + ".outputX", shp.attr("visibility"))

        fkvis2_node = node.createReverseNode(self.blend2_att)
        for shp in self.fk2_ctl.getShapes():
            pm.connectAttr(fkvis2_node + ".outputX", shp.attr("visibility"))

        # ik
        for shp in self.upv_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.ikcns_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.ik_ctl.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))
        for shp in self.line_ref.getShapes():
            pm.connectAttr(self.blend_att, shp.attr("visibility"))

        # jnt ctl
        for ctl in (self.div_ctls):
            for shp in ctl.getShapes():
                pm.connectAttr(self.jntctl_vis_att, shp.attr("visibility"))

        # Controls ROT order -----------------------------------
        attribute.setRotOrder(self.fk0_ctl, "YZX")
        attribute.setRotOrder(self.fk1_ctl, "XYZ")
        attribute.setRotOrder(self.fk2_ctl, "YZX")
        attribute.setRotOrder(self.ik_ctl, "XYZ")

        # IK Solver -----------------------------------------
        out = [self.bone0, self.bone1, self.ctrn_loc, self.eff_loc]

        mg_node = applyop.gear_ikfk2bone_op(out,
                                           self.root,
                                           self.ik_ref,
                                           self.upv_ctl,
                                           self.fk0_mtx,
                                           self.fk1_mtx,
                                           self.fk2_mtx,
                                           self.length0,
                                           self.length1,
                                           self.negate)

        outEff_dm = mg_node.listConnections(c=True)[-1][1]

        pm.connectAttr(self.blend_att, mg_node + ".blend")
        if self.negate:
            mulVal = -1
        else:
            mulVal = 1
        node.createMulNode(self.roll_att, mulVal, mg_node + ".roll")
        # pm.connectAttr(self.roll_att, o_node + ".roll")
        pm.connectAttr(self.scale_att, mg_node + ".scaleA")
        pm.connectAttr(self.scale_att, mg_node + ".scaleB")
        pm.connectAttr(self.maxstretch_att, mg_node + ".maxstretch")
        pm.connectAttr(self.slide_att, mg_node + ".slide")
        pm.connectAttr(self.softness_att, mg_node + ".softness")
        pm.connectAttr(self.reverse_att, mg_node + ".reverse")
        # update issue on effector scale interpol, disconnect for stability

        # auto upvector -------------------------------------

        if self.negate:
            o_node = applyop.aimCns(self.upv_auv,
                                    self.ik_ctl,
                                    axis="-xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.upv_auv,
                                    maintainOffset=False)
        else:
            o_node = applyop.aimCns(self.upv_auv,
                                    self.ik_ctl,
                                    axis="xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.upv_auv,
                                    maintainOffset=False)

        o_node = applyop.gear_mulmatrix_op(
            self.upv_auv.attr("worldMatrix"),
            self.upv_mtx.attr("parentInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(o_node + ".output", dm_node + ".inputMatrix")
        pb_node = pm.createNode("pairBlend")
        pb_node.attr("rotInterpolation").set(1)
        pm.connectAttr(dm_node + ".outputTranslate", pb_node + ".inTranslate2")
        pm.connectAttr(dm_node + ".outputRotate", pb_node + ".inRotate2")
        pm.connectAttr(pb_node + ".outRotate", self.upv_mtx.attr("rotate"))
        pm.connectAttr(pb_node + ".outTranslate",
                       self.upv_mtx.attr("translate"))
        pm.connectAttr(self.auv_att, pb_node + ".weight")

        # fk0 mtx connection
        o_node = applyop.gear_mulmatrix_op(
            self.fk0_roll_ctl.attr("worldMatrix"),
            self.fk0_mtx.attr("parentInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(o_node + ".output", dm_node + ".inputMatrix")
        pm.connectAttr(dm_node + ".outputTranslate",
                       self.fk0_mtx.attr("translate"))
        pm.connectAttr(dm_node + ".outputRotate", self.fk0_mtx.attr("rotate"))

        o_node = applyop.gear_mulmatrix_op(
            self.fk1_ref.attr("worldMatrix"),
            self.fk1_loc.attr("parentInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(o_node + ".output", dm_node + ".inputMatrix")
        pm.connectAttr(dm_node + ".outputTranslate",
                       self.fk1_loc.attr("translate"))
        pm.connectAttr(dm_node + ".outputRotate", self.fk1_loc.attr("rotate"))

        # connect forfarm twist
        pm.connectAttr(self.fk1_roll_ctl.attr("rotate"),
                       self.fk2_npo.attr("rotate"))
        # fk2_loc position constraint to eff_npo----------------------
        outEff_dm.attr("outputTranslate") >> self.fk2_loc.attr("translate")
        o_node = applyop.gear_mulmatrix_op(
            self.bone1.attr("worldMatrix"),
            self.fk2_loc.attr("parentInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(o_node + ".output", dm_node + ".inputMatrix")
        pm.connectAttr(dm_node + ".outputRotate", self.fk2_loc.attr("rotate"))

        # hand ikfk blending from fk ref to ik ref (serious bugfix)--------
        intM_node = applyop.gear_intmatrix_op(
            self.fk_ref.attr("worldMatrix"),
            self.ik_ref.attr("worldMatrix"),
            self.blend2_att)
        o_node = applyop.gear_mulmatrix_op(
            intM_node.attr("output"),
            self.eff_loc.attr("parentInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(o_node + ".output", dm_node + ".inputMatrix")
        dm_node.attr("outputScale") >> self.eff_loc.attr("scale")
        dm_node.attr("outputRotate") >> self.eff_loc.attr("rotate")

        # Twist references ---------------------------------
        # use p constraint to counter mirror effect meg scaling
        pm.parentConstraint(self.mid_ctl,
                            self.mid_loc,
                            mo=True)


        o_node = applyop.gear_mulmatrix_op(
            self.eff_loc.attr("worldMatrix"),
            self.tws3_npo.attr("parentInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(o_node + ".output", dm_node + ".inputMatrix")

        pm.connectAttr(dm_node + ".outputTranslate",
                       self.tws3_npo.attr("translate"))

        pm.connectAttr(dm_node + ".outputRotate",
                       self.tws3_npo.attr("rotate"))

        attribute.setRotOrder(self.tws3_rot, "XYZ")

        # elbow thickness connection
        if self.negate:
            o_node = node.createMulNode(
                [self.elbow_thickness_att, self.elbow_thickness_att],
                [0.5, -0.5, 0],
                [self.tws1_loc + ".translateX", self.tws2_loc + ".translateX"])
        else:
            o_node = node.createMulNode(
                [self.elbow_thickness_att, self.elbow_thickness_att],
                [-0.5, 0.5, 0],
                [self.tws1_loc + ".translateX", self.tws2_loc + ".translateX"])


        pm.connectAttr(self.armpit_roll_att, self.tws0_roll + ".rotateX")

        # Roll Shoulder--use aimconstraint withour uovwctor to solve
        # the stable twist
        # setup all aim constraints
        #tws0_loc aim
        if self.negate:
            o_node = applyop.aimCns(self.tws0_loc,
                                    self.tan1_npo,
                                    axis="-xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.mid_loc,
                                    maintainOffset=False)
        else:
            o_node = applyop.aimCns(self.tws0_loc,
                                    self.tan1_npo,
                                    axis="xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.mid_loc,
                                    maintainOffset=False)
        #tws3_loc aim
        if self.negate:
            o_node = applyop.aimCns(self.tws3_loc,
                                    self.tan2_npo,
                                    axis="xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.mid_loc,
                                    maintainOffset=False)
        else:
            o_node = applyop.aimCns(self.tws3_loc,
                                    self.tan2_npo,
                                    axis="-xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.mid_loc,
                                    maintainOffset=False)
        #tws1_loc aim
        if self.negate:
            o_node = applyop.aimCns(self.tws1_loc,
                                    self.root,
                                    axis="xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.mid_loc,
                                    maintainOffset=False)
        else:
            o_node = applyop.aimCns(self.tws1_loc,
                                    self.root,
                                    axis="-xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.mid_loc,
                                    maintainOffset=False)
        pm.disconnectAttr(o_node+".constraintRotateX",self.tws1_loc.rx)
        pm.disconnectAttr(o_node+".constraintRotateY",self.tws1_loc.ry)
        pm.disconnectAttr(o_node+".constraintRotateZ",self.tws1_loc.rz)
        pb_node = pm.createNode("pairBlend",n=self.getName("roundness0_pb"))
        pb_node.attr("rotInterpolation").set(1)
        pm.connectAttr(o_node+".constraintRotate",pb_node+".inRotate1")
        # if self.negate:
        #   pb_node.inRotateZ2.set(-180)
        pm.connectAttr(o_node+".message",pb_node+".inTranslate2")
        pm.connectAttr(pb_node+".outRotate",self.tws1_loc+".rotate")
        pm.connectAttr(self.roundness0_att,pb_node+".weight")
        #tws2_loc_aim
        if self.negate:
            o_node = applyop.aimCns(self.tws2_loc,
                                    self.eff_loc,
                                    axis="-xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.mid_loc,
                                    maintainOffset=False)
        else:
            o_node = applyop.aimCns(self.tws2_loc,
                                    self.eff_loc,
                                    axis="xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.mid_loc,
                                    maintainOffset=False)
        pm.disconnectAttr(o_node+".constraintRotateX",self.tws2_loc.rx)
        pm.disconnectAttr(o_node+".constraintRotateY",self.tws2_loc.ry)
        pm.disconnectAttr(o_node+".constraintRotateZ",self.tws2_loc.rz)
        pb_node = pm.createNode("pairBlend",n=self.getName("roundness1_pb"))
        pb_node.attr("rotInterpolation").set(1)
        pm.connectAttr(o_node+".constraintRotate",pb_node+".inRotate1")
        pm.connectAttr(o_node+".message",pb_node+".inTranslate2")
        pm.connectAttr(pb_node+".outRotate",self.tws2_loc+".rotate")
        pm.connectAttr(self.roundness1_att,pb_node+".weight")
        # freeform tangent ctl setup

        #tws0 rot tangent ctl aim
        if self.negate:
            o_node = applyop.aimCns(self.tws0_rot,
                                    self.tan0_ctl,
                                    axis="-xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.tws0_loc,
                                    maintainOffset=False)
        else:
            o_node = applyop.aimCns(self.tws0_rot,
                                    self.tan0_ctl,
                                    axis="xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.tws0_loc,
                                    maintainOffset=False)  
        #tws1 rot tangent ctl aim
        if self.negate:
            o_node = applyop.aimCns(self.tws1_rot,
                                    self.tan1_ctl,
                                    axis="xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.tws1_loc,
                                    maintainOffset=False)
        else:
            o_node = applyop.aimCns(self.tws1_rot,
                                    self.tan1_ctl,
                                    axis="-xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.tws1_loc,
                                    maintainOffset=False)
		#tws2 rot tangent ctl aim
        if self.negate:
            o_node = applyop.aimCns(self.tws2_rot,
                                    self.tan2_ctl,
                                    axis="-xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.tws1_loc,
                                    maintainOffset=False)
        else:
            o_node = applyop.aimCns(self.tws2_rot,
                                    self.tan2_ctl,
                                    axis="xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.tws1_loc,
                                    maintainOffset=False)
        #tws3 rot tangent ctl aim
        if self.negate:
            o_node = applyop.aimCns(self.tws3_rot,
                                    self.tan3_ctl,
                                    axis="xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.tws1_loc,
                                    maintainOffset=False)
        else:
            o_node = applyop.aimCns(self.tws3_rot,
                                    self.tan3_ctl,
                                    axis="-xy",
                                    wupType=4,
                                    wupVector=[0, 1, 0],
                                    wupObject=self.tws1_loc,
                                    maintainOffset=False)
        for ctl in (self.tan_ctls):
            for shp in ctl.getShapes():
                pm.connectAttr(self.tan_vis_att, shp.attr("visibility"))
        # Volume -------------------------------------------
        distA_node = node.createDistNode(self.root, self.mid_ctl)
        distB_node = node.createDistNode(self.mid_ctl, self.eff_loc)
        add_node = node.createAddNode(distA_node + ".distance",
                                      distB_node + ".distance")
        div_node = node.createDivNode(add_node + ".output",
                                      self.root.attr("sx"))

        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(self.root.attr("worldMatrix"), dm_node + ".inputMatrix")

        div_node2 = node.createDivNode(div_node + ".outputX",
                                       dm_node + ".outputScaleX")
        self.volDriver_att = div_node2 + ".outputX"

        # set up tangent npo pos using distA and B-------------------------------


        # up tangent-----------------------------------
        if self.negate:
          factor = [-0.3333,0.3333]
        else:
          factor = [0.3333,-0.3333] 
        div_node = node.createDivNode(distA_node + ".distance",
                                       dm_node + ".outputScaleX")

        mul_node = node.createMulNode(div_node + ".outputX", factor[0])
        pm.connectAttr(mul_node + ".outputX", self.tan0_npo.tx)
        mul_node = node.createMulNode(div_node + ".outputX", factor[1])
        pm.connectAttr(mul_node + ".outputX", self.tan1_npo.tx)
        # dn tangent ----------------------------------
        div_node = node.createDivNode(distB_node + ".distance",
                                       dm_node + ".outputScaleX")

        mul_node = node.createMulNode(div_node + ".outputX", factor[0])
        pm.connectAttr(mul_node + ".outputX", self.tan2_npo.tx)
        mul_node = node.createMulNode(div_node + ".outputX", factor[1])
        pm.connectAttr(mul_node + ".outputX", self.tan3_npo.tx)

        # tangent mgear roll spline scl connection
        # t0 
        o_node = applyop.gear_mulmatrix_op(
            self.tan0_ctl.attr("worldMatrix"),
            self.tws0_loc.attr("worldInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix",n=self.getName("tan0Localpos_dm"))
        distTan_node = pm.createNode("distanceBetween", n=self.getName("tan0_dist"))
        pm.connectAttr(o_node+".output", dm_node + ".inputMatrix")
        pm.connectAttr(dm_node+".outputTranslate", distTan_node + ".point1")
        pm.connectAttr(self.tws0_loc+".message", distTan_node + ".point2")
        mul_node = node.createMulNode(distTan_node + ".distance", 0.5)
        pm.connectAttr(mul_node+".outputX",self.tws0_rot.sx)

        # t1
        o_node = applyop.gear_mulmatrix_op(
            self.tan1_ctl.attr("worldMatrix"),
            self.tws1_loc.attr("worldInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix",n=self.getName("tan1Localpos_dm"))
        distTan_node = pm.createNode("distanceBetween", n=self.getName("tan1_dist"))
        pm.connectAttr(o_node+".output", dm_node + ".inputMatrix")
        pm.connectAttr(dm_node+".outputTranslate", distTan_node + ".point1")
        pm.connectAttr(self.tws1_loc+".message", distTan_node + ".point2")
        mul_node = node.createMulNode(distTan_node + ".distance", 0.5)
        pm.connectAttr(mul_node+".outputX",self.tws1_rot.sx)        

        # t2
        o_node = applyop.gear_mulmatrix_op(
            self.tan2_ctl.attr("worldMatrix"),
            self.tws2_loc.attr("worldInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix",n=self.getName("tan2Localpos_dm"))
        distTan_node = pm.createNode("distanceBetween", n=self.getName("tan2_dist"))
        pm.connectAttr(o_node+".output", dm_node + ".inputMatrix")
        pm.connectAttr(dm_node+".outputTranslate", distTan_node + ".point1")
        pm.connectAttr(self.tws2_loc+".message", distTan_node + ".point2")
        mul_node = node.createMulNode(distTan_node + ".distance", 0.5)
        pm.connectAttr(mul_node+".outputX",self.tws2_rot.sx)

        # t3
        o_node = applyop.gear_mulmatrix_op(
            self.tan3_ctl.attr("worldMatrix"),
            self.tws3_loc.attr("worldInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix",n=self.getName("tan2Localpos_dm"))
        distTan_node = pm.createNode("distanceBetween", n=self.getName("tan2_dist"))
        pm.connectAttr(o_node+".output", dm_node + ".inputMatrix")
        pm.connectAttr(dm_node+".outputTranslate", distTan_node + ".point1")
        pm.connectAttr(self.tws3_loc+".message", distTan_node + ".point2")
        mul_node = node.createMulNode(distTan_node + ".distance", 0.5)
        pm.connectAttr(mul_node+".outputX",self.tws3_rot.sx)   
        # Divisions ----------------------------------------
        # div mid constraint to mid ctl
        o_node = applyop.gear_mulmatrix_op(
            self.mid_ctl.attr("worldMatrix"),
            self.div_mid.attr("parentInverseMatrix"))
        dm_node = pm.createNode("decomposeMatrix")
        pm.connectAttr(o_node + ".output", dm_node + ".inputMatrix")
        pm.connectAttr(dm_node + ".outputTranslate",
                       self.div_mid.attr("translate"))
        pb_node = node.createPairBlend(self.div_cnsUp[-1],self.div_cnsDn[0])
        pb_node.attr("rotInterpolation").set(1)
        


        pm.connectAttr(pb_node + ".outRotate",
                       self.div_mid.attr("rotate"))

        # at 0 or 1 the division will follow exactly the rotation of the
        # controler.. and we wont have this nice tangent + roll
        # linear scaling percentage (1) to effector (2) to elbow
        scl_1_perc = []
        scl_2_perc = []

        for i, div_cnsUp in enumerate(self.div_cnsUp):

            if i < (self.settings["div0"] + 1):
                perc = i / (self.settings["div0"] + 1.0)
            elif i < (self.settings["div0"] + 2):
                perc = .95

            perc = max(.001, min(.99, perc))

            # Roll
            if self.negate:
                o_node = applyop.gear_rollsplinekine_op(
                    div_cnsUp, [self.tws1_rot, self.tws0_rot], 1 - perc, 20)

            else:
                o_node = applyop.gear_rollsplinekine_op(
                    div_cnsUp, [self.tws0_rot, self.tws1_rot], perc, 20)

            pm.connectAttr(self.resample_att, o_node + ".resample")
            pm.connectAttr(self.absolute_att, o_node + ".absolute")

            scl_1_perc.append(perc / 2)
            scl_2_perc.append(perc)
        scl_1_perc.append(0.5)
        scl_2_perc.append(1)
        for i, div_cnsDn in enumerate(self.div_cnsDn):

            if i == (0):
                perc = .05
            elif i < (self.settings["div1"] + 1):
                perc = i / (self.settings["div1"] + 1.0)
            elif i < (self.settings["div1"] + 2):
                perc = .95

            perc = max(.001, min(.990, perc))

            # Roll
            if self.negate:
                o_node = applyop.gear_rollsplinekine_op(
                    div_cnsDn, [self.tws3_rot, self.tws2_rot], 1 - perc, 20)

            else:
                o_node = applyop.gear_rollsplinekine_op(
                    div_cnsDn, [self.tws2_rot, self.tws3_rot], perc, 20)
            pm.connectAttr(self.resample_att, o_node + ".resample")
            pm.connectAttr(self.absolute_att, o_node + ".absolute")

            scl_1_perc.append(perc / 2 + 0.5)
            scl_2_perc.append(1 - perc)
        # Squash n Stretch
        for i, div_cns in enumerate(self.div_cns):
            o_node = applyop.gear_squashstretch2_op(
                div_cns, None, pm.getAttr(self.volDriver_att), "x")
            pm.connectAttr(self.volume_att, o_node + ".blend")
            pm.connectAttr(self.volDriver_att, o_node + ".driver")
            pm.connectAttr(self.st_att[i], o_node + ".stretch")
            pm.connectAttr(self.sq_att[i], o_node + ".squash")
            # get the first mult_node after sq op
            mult_node = pm.listHistory(o_node, future=True)[1]
            # linear blend effector scale
         
            bc_node = pm.createNode("mgear_linearInterpolate3DvectorNode",n=self.getName("mgBlendEffScl"))
            bc_node.setAttr("vectorBX", 1)
            bc_node.setAttr("vectorAX", 1)
            bc_node.setAttr("vectorAY", 1)
            bc_node.setAttr("vectorAZ", 1)
            bc_node.setAttr("blend", scl_1_perc[i])
            pm.connectAttr(self.eff_loc.attr("scaleY"), bc_node + ".vectorBY")
            pm.connectAttr(self.root.attr("scaleY"), bc_node + ".vectorAY")
            # linear blend mid scale
            bc_node2 = pm.createNode("mgear_linearInterpolate3DvectorNode",n=self.getName("mgBlendMidScl"))
            bc_node2.setAttr("vectorBX", 1)
            bc_node2.setAttr("vectorAX", 1)
            bc_node2.setAttr("vectorAY", 1)
            bc_node2.setAttr("vectorAZ", 1)
            bc_node2.setAttr("blend", scl_2_perc[i])
            pm.connectAttr(self.mid_ctl.attr("scaleY"), bc_node2 + ".vectorBY")
            pm.connectAttr(self.root.attr("scaleY"), bc_node2 + ".vectorAY")


            # mid_ctl scale * effector scale
            mult_node2 = pm.createNode("multiplyDivide",n=self.getName("linearSclCombine"))
            pm.connectAttr(bc_node2 + ".outVector", mult_node2 + ".input1")
            pm.connectAttr(bc_node + ".outVector", mult_node2 + ".input2")
            # plug to sq scale
            pm.connectAttr(mult_node2 + ".outputY", mult_node + ".input2Y")
            pm.connectAttr(mult_node2 + ".outputY", mult_node + ".input2Z")
        # match IK/FK ref
        pm.connectAttr(self.bone0.attr("rotate"),
                       self.match_fk0.attr("rotate"))
        pm.connectAttr(self.bone0.attr("translate"),
                       self.match_fk0.attr("translate"))
        pm.connectAttr(self.bone1.attr("rotate"),
                       self.match_fk1.attr("rotate"))
        pm.connectAttr(self.bone1.attr("translate"),
                       self.match_fk1.attr("translate"))

        return

    # =====================================================
    # CONNECTOR
    # =====================================================
    def setRelation(self):
        """Set the relation beetween object from guide to rig"""
        self.relatives["root"] = self.div_cns[0]
        self.relatives["elbow"] = self.div_cns[self.settings["div0"] + 2]
        self.relatives["wrist"] = self.div_cns[-1]
        self.relatives["eff"] = self.eff_loc

        self.jointRelatives["root"] = 0
        self.jointRelatives["elbow"] = self.settings["div0"] + 2
        self.jointRelatives["wrist"] = len(self.div_cns) - 1
        self.jointRelatives["eff"] = -1

        self.controlRelatives["root"] = self.fk0_ctl
        self.controlRelatives["elbow"] = self.fk1_ctl
        self.controlRelatives["wrist"] = self.fk2_ctl
        self.controlRelatives["eff"] = self.fk2_ctl

        self.aliasRelatives["eff"] = "hand"

    def addConnection(self):
        """Add more connection definition to the set"""
        self.connections["shoulder_01"] = self.connect_shoulder_01
    def connect_standard(self):
        """standard connection definition for the component"""
        self.connect_standardWithIkRef()
        # fk isolation connection
        self.connect_standardWithRotRef(self.settings["fkrefarray"],
                                        self.fk_cns)

    def connect_shoulder_01(self):
        """ Custom connection to be use with shoulder 01 component"""
        self.connect_standard()
        pm.parent(self.tws0_npo,
                  self.upv_org,
                  self.parent_comp.ctl)


