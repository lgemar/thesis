#if UNITY_5
using UnityEngine;
using UnityEditor;

using System.IO;
using System.Linq;
using System.Text.RegularExpressions;
using System.Collections.Generic;
using System;

class OpenCVForUnityMenuItem : MonoBehaviour
{

    /// <summary>
    /// Sets the plugin import settings.
    /// </summary>
    [MenuItem("Tools/OpenCV for Unity/Set Plugin Import Settings")]
    static void SetPluginImportSettings()
    {
        //Android
        SetPlugins(new string[] { "Assets/OpenCVForUnity/Plugins/Android/opencvforunity.jar" }, null,
           new Dictionary<BuildTarget, Dictionary<string, string>>() { { BuildTarget.Android, null } });
        SetPlugins(GetPluginFilePaths("Assets/OpenCVForUnity/Plugins/Android/libs/armeabi-v7a"), null,
new Dictionary<BuildTarget, Dictionary<string, string>>(){{BuildTarget.Android,new Dictionary<string, string> ()
                {{"CPU", "ARMv7"}
                }}});
        SetPlugins(GetPluginFilePaths("Assets/OpenCVForUnity/Plugins/Android/libs/x86"), null,
           new Dictionary<BuildTarget, Dictionary<string, string>>(){{BuildTarget.Android,new Dictionary<string, string> ()
                {{"CPU", "x86"}
                }}});


        //iOS
        SetPlugins(new string[] { "Assets/OpenCVForUnity/Plugins/iOS/opencv2.framework" }, null,
new Dictionary<BuildTarget, Dictionary<string, string>>() { { BuildTarget.iOS, null } });
        SetPlugins(GetPluginFilePaths("Assets/OpenCVForUnity/Plugins/iOS"), null,
                         new Dictionary<BuildTarget, Dictionary<string, string>>() { { BuildTarget.iOS, null } });


        //OSX
        SetPlugins(new string[] { "Assets/OpenCVForUnity/Plugins/opencvforunity.bundle" }, new Dictionary<string, string>() { { "CPU", "AnyCPU" }, { "OS", "OSX" } },
new Dictionary<BuildTarget, Dictionary<string, string>>(){{BuildTarget.StandaloneOSXIntel,new Dictionary<string, string> ()
                {{"CPU", "x86"}
                }},{BuildTarget.StandaloneOSXIntel64,new Dictionary<string, string> ()
                {{"CPU", "x86_64"}
                }},{BuildTarget.StandaloneOSXUniversal,new Dictionary<string, string> ()
                {{"CPU", "AnyCPU"}
                }}});


        //Windows
        SetPlugins(new string[] { "Assets/OpenCVForUnity/Plugins/x86/opencvforunity.dll" }, new Dictionary<string, string>() { { "CPU", "x86" }, { "OS", "Windows" } },
                   new Dictionary<BuildTarget, Dictionary<string, string>>(){{BuildTarget.StandaloneWindows,new Dictionary<string, string> ()
                        {{"CPU", "x86"}
                }}});

        SetPlugins(new string[] { "Assets/OpenCVForUnity/Plugins/x86_64/opencvforunity.dll" }, new Dictionary<string, string>() { { "CPU", "x86_64" }, { "OS", "Windows" } },
        new Dictionary<BuildTarget, Dictionary<string, string>>(){{BuildTarget.StandaloneWindows64,new Dictionary<string, string> ()
                {{"CPU", "x86_64"}
                }}});

		
    }

    /// <summary>
    /// Gets the plugin file paths.
    /// </summary>
    /// <returns>The plugin file paths.</returns>
    /// <param name="folderPath">Folder path.</param>
    static string[] GetPluginFilePaths(string folderPath)
    {
        Regex reg = new Regex(".meta$|.DS_Store");
		try {
			return Directory.GetFiles (folderPath).Where (f => !reg.IsMatch (f)).ToArray ();
		} catch (Exception ex) {
			Debug.LogWarning("SetPluginImportSettings Faild :" + ex);
			return null;
		}
    }

    /// <summary>
    /// Sets the plugins.
    /// </summary>
    /// <param name="files">Files.</param>
    /// <param name="editorSettings">Editor settings.</param>
    /// <param name="settings">Settings.</param>
    static void SetPlugins(string[] files, Dictionary<string, string> editorSettings, Dictionary<BuildTarget, Dictionary<string, string>> settings)
    {
		if(files == null)return;

        foreach (string item in files)
        {

            PluginImporter pluginImporter = PluginImporter.GetAtPath(item) as PluginImporter;

            if (pluginImporter != null)
            {

                pluginImporter.SetCompatibleWithAnyPlatform(false);
                pluginImporter.SetCompatibleWithEditor(false);


                if (editorSettings != null)
                {
                    pluginImporter.SetCompatibleWithEditor(true);

                    foreach (KeyValuePair<string, string> pair in editorSettings)
                    {
                        pluginImporter.SetEditorData(pair.Key, pair.Value);
                    }
                }

                if (settings != null)
                {
                    foreach (KeyValuePair<BuildTarget, Dictionary<string, string>> settingPair in settings)
                    {

                        pluginImporter.SetCompatibleWithPlatform(settingPair.Key, true);
                        if (settingPair.Value != null)
                        {
                            foreach (KeyValuePair<string, string> pair in settingPair.Value)
                            {
                                pluginImporter.SetPlatformData(settingPair.Key, pair.Key, pair.Value);
                            }
                        }

                    }
                }
                else
                {
                    foreach (BuildTarget target in Enum.GetValues(typeof(BuildTarget)))
                    {
                        pluginImporter.SetCompatibleWithPlatform(target, false);
                    }
                }


                pluginImporter.SaveAndReimport();

                Debug.Log("SetPluginImportSettings Success :" + item);
            }
            else
            {
                Debug.LogWarning("SetPluginImportSettings Faild :" + item);
            }
        }
    }
}
#endif