using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using System.IO;
using System.Drawing;
using Microsoft.Win32;
using System.Threading;
using System.ComponentModel;
using LOC;
using LOC.Image;
using LOC.FeatureMatching;
using LOC.Photogrammetry;
using LOC.Photogrammetry.CoordinateTransform;


namespace HW3
{
    /// <summary>
    /// MainWindow.xaml 的互動邏輯
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }

        LOCImage OImage = null;
        LOCImage ProcessedImage = null;
        
        string ProcessFileName;
        int dataNum;
        int dataNow;
        string[] files;
        string FileNoExt;
        List<string> FileNoExt2 = new List<string>();
        private void Load_and_Calibrate_Click(object sender, RoutedEventArgs e)
        {
            
            string path = @"D:\四下\近景攝影測量\HW3_code and calibrated process\hw3\photo\test";
             files = Directory.GetFiles(path, "*JPG");
            dataNum = files.Length;
            

            pb.Minimum = 1;
            pb.Maximum = dataNum;
            
            BackgroundWorker bw = new BackgroundWorker();
            bw.WorkerSupportsCancellation = true;
            bw.WorkerReportsProgress = true;

            /// add the event handler for each progress
            bw.DoWork += new DoWorkEventHandler(DoWork);
            bw.ProgressChanged +=
                new ProgressChangedEventHandler(DuringWork);
            bw.RunWorkerCompleted +=
                new RunWorkerCompletedEventHandler(AfterWork);

            /// start the background work
            bw.RunWorkerAsync();


        }
        void DoWork(object sender, DoWorkEventArgs e)
        {
            dataNow = 0;
            BackgroundWorker w = sender as BackgroundWorker;
            foreach (string filename in files)
            {
                dataNow++;
                FileNoExt = System.IO.Path.GetFileNameWithoutExtension(filename);
                FileNoExt2.Add(FileNoExt);
                LOCImage OImage = new LOCImage(filename, Int32Rect.Empty);

                //改正
                Calibrate CA = new Calibrate(3);
                CA.Coeffs = new float[7];
                CA.Coeffs[0] = -0.0992f;// - 0.0760f;
                CA.Coeffs[1] = 0.1332f;// 0.1307f;
                CA.Coeffs[2] = 1.30096e-04f;// 1.3632e-04f;
                CA.Coeffs[3] = 3.84227e-08f;// - 2.0510e-07f;
                CA.Coeffs[4] = -5.48498e-10f;// 3.4733e-09f;
                CA.Coeffs[5] = 3.9712e-06f;// - 3.0489e-06f;
                CA.Coeffs[6] = -2.6208e-05f;// - 2.7386e-05f;
                ProcessedImage = new LOCImage(OImage.Width, OImage.Height, 72, 72, PixelFormats.Bgr24, null);
                InOrietation IO = new InOrietation(OImage.Width, OImage.Height, 32.0585f, 0.0037f);
                for (int i = 0; i < OImage.Width; i++)
                {
                    int Index = 0;
                    for (int j = 0; j < OImage.Height; j++)
                    {
                        Index = (j * OImage.Width + i) * 3;
                        List<ImagePoints.Coordinate> POINT = new List<ImagePoints.Coordinate>
                        {new ImagePoints.Coordinate(i,j,CoordinateFormat.Pixel )};
                        
                        
                        CA.Transform(POINT[0], IO);
                        for (int k = 0; k < ProcessedImage.NoBands; k++)
                        {
                            ProcessedImage.ByteData[Index + k] = (byte)Interpolation.Bilinear(OImage, CA.TransformPt[0], CA.TransformPt[1], k);                           
                        }
                    }

                }

                //ProcessedImage = OImage;//不做處理

                 ProcessFileName = "D:\\四下\\近景攝影測量\\HW3_my calibrate\\0607result\\calibrated images\\calibrated" + FileNoExt + ".tif";
                ProcessedImage.Save(ProcessFileName, ImageFormat.Tiff);
            
                w.ReportProgress(dataNow);

            }

        }

        void DuringWork(
            object sender, ProgressChangedEventArgs e)
        {
            /// reflect the change of progress in UI
            Load_and_Calibrate.Content =
                e.ProgressPercentage.ToString() + "/" + dataNum.ToString();
            pb.Value = e.ProgressPercentage;
            lb.Items.Add("Image " + e.ProgressPercentage.ToString());
        }

        void AfterWork(
            object sender, RunWorkerCompletedEventArgs e)
        {
            /// reflect the result after background work
            if (e.Cancelled == true)
            {
                Load_and_Calibrate.Content = "Canceled!";
            }
            else if (!(e.Error == null))
            {
                Load_and_Calibrate.Content = ("Error: " + e.Error.Message);
            }
            else
            {
                Load_and_Calibrate.Content = "Done!";
            }
        }
        private void Lb_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            int itemindex = lb.SelectedIndex;
            tb.Text = "   You selected Image " + (itemindex+1).ToString() + ".";          
            string originalfilename = files[itemindex];
            string processedfilename = "D:\\四下\\近景攝影測量\\HW3_my calibrate\\0607result\\calibrated images\\calibrated" + FileNoExt2[itemindex] + ".tif";
            original.Source = new BitmapImage(new Uri(originalfilename));
            processed.Source = new BitmapImage(new Uri(processedfilename));
        }
    }
}
