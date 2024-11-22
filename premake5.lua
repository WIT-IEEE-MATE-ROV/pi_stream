-- premake5.lua
workspace "pi_stream"
   configurations { "Debug", "Release" }

project "pi_stream"
   kind "ConsoleApp"
   language "C++"
   targetdir "bin/%{cfg.buildcfg}"

   files { "src/**.h", "src/**.hpp", "src/**.cpp" }

   includedirs { 
      "/usr/include/opencv4/opencv2/",
      "/usr/include/",
      
   }
   
   links {
      "opencv_core",
      "opencv_highgui",
      "opencv_videoio",
      "zmq"
   }

   filter "configurations:Debug"
      defines { "DEBUG" }
      symbols "On"

   filter "configurations:Release"
      defines { "NDEBUG" }
      optimize "On"
