import os
import sys
import excons
import excons.config
import excons.tools.maya as maya


maya.SetupMscver()
env = excons.MakeBaseEnv()


version = (2, 5, 25)
versionstr = "%d.%d.%d" % version
platname = {"win32": "windows", "darwin": "osx"}.get(sys.platform, "linux")
outprefix = "platforms/%s/%s/%s/plug-ins" % (maya.Version(nice=True), platname, excons.arch_dir)

outdir = excons.OutputBaseDirectory()

gen = excons.config.AddGenerator(env, "mgear", {"MGEAR_VERSION": "[%d, %d, %d]" % version,
                                                "MGEAR_MAJMIN_VERSION": "%d.%d" % (version[0], version[1])})

mgearinit = gen("scripts/mgear/__init__.py", "scripts/mgear/__init__.py.in")
mgearmod = gen("mGear.mod", "mGear.mod.in")
mgearpy = filter(lambda x: not os.path.basename(x).startswith("__init__.py"), excons.glob("scripts/mgear/*"))
qtpy = ["Qtdotpy/Qt.py"]
NoClean(mgearinit + mgearmod)

defines = []
if sys.platform == "win32":
   defines.append("NOMINMAX")

def CVWrapSetup(env):
   if sys.platform == "win32":
      env.Append(CCFLAGS=["/arch:AVX"])
   else:
      env.Append(CCFLAGS=["-mavx"])

targets = [
   {
      "name": "mgear_core",
      "type": "install",
      "desc": "mgear core python modules",
      "install": {"scripts": excons.glob("scripts/*.py"),
                  "scripts/mgear": mgearpy + mgearinit,
                  "scripts/mgear/vendor": qtpy,
                  "tests": excons.glob("tests/*.py"),
                  "": mgearmod}
   },
   {
      "name": "mgear_solvers",
      "type": "dynamicmodule",
      "desc": "mgear solvers plugin",
      "prefix": outprefix,
      "bldprefix": maya.Version(),
      "ext": maya.PluginExt(),
      "defs": defines,
      "incdirs": ["src"],
      "srcs": excons.glob("src/*.cpp"),
      "custom": [maya.Require]
   },
   {
      "name": "cvwrap",
      "type": "dynamicmodule",
      "desc": "wrap deformer plugin",
      "prefix": outprefix,
      "bldprefix": maya.Version(),
      "ext": maya.PluginExt(),
      "defs": defines,
      "incdirs": ["src"],
      "srcs": excons.glob("cvwrap/src/*.cpp"),
      "custom": [maya.Require, CVWrapSetup],
      "libs": ([] if maya.Version(asString=False) < 201600 else ["clew"]),
      "install": {"scripts": excons.glob("cvwrap/scripts/*")}
   }
]

excons.AddHelpTargets(mgear="mgear maya framework (mgear_core, mgear_solvers, cvwrap)")

td = excons.DeclareTargets(env, targets)

env.Alias("mgear", [td["mgear_core"], td["mgear_solvers"], td["cvwrap"]])

td["python"] = filter(lambda x: os.path.splitext(str(x))[1] != ".mel", Glob(outdir + "/scripts/*"))
td["scripts"] = Glob(outdir + "/scripts/*.mel")

pluginsdir = "/plug-ins/%s/%s" % (maya.Version(nice=True), excons.EcosystemPlatform())

ecodirs = {"mgear_solvers": pluginsdir,
           "cvwrap": pluginsdir,
           "python": "/python",
           "scripts": "/scripts"}

excons.EcosystemDist(env, "mgear.env", ecodirs, version=versionstr, targets=td)

Default(["mgear"])
