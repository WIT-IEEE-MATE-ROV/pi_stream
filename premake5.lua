-- premake5.lua
workspace "pi_stream"
   architecture "x64"
   configurations { "Debug", "Release" }
   platforms {"x64", "ARM", "ARM64" } 

project "pi_stream"
   kind "ConsoleApp"
   language "C++"

   targetdir "bin/%{cfg.buildcfg}"

   files { "src/**.h", "src/**.hpp", "src/**.cpp", "include/**.h" }

   includedirs { 
      "/usr/include/opencv4/",
      "/usr/include/",
      "include/"
   }
   
   links {
      "opencv_core",
      "opencv_imgproc",
      "opencv_imgcodecs",
      "opencv_features2d",
      "opencv_highgui",
      "opencv_videoio",
   }

   filter "platforms:x64"
      architecture "x64"

   filter "platforms:ARM"
      architecture "ARM"

   filter "platforms:ARM64"
      architecture "ARM64"


   filter "configurations:Debug"
      defines { "DEBUG" }
      symbols "On"

   filter "configurations:Release"
      defines { "NDEBUG" }
      optimize "On"
