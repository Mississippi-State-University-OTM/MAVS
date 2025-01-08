# Terrain Roughness in MAVS
Terrain roughness is an important factor [influencing vehicle mobility](https://apps.dtic.mil/sti/pdfs/ADA178359.pdf). This manifests itself primarily as a speed-limiter caused by [driver discomfort](https://apps.dtic.mil/sti/pdfs/ADA282588.pdf), although roughness has also been considered in the context of [unmanned and autonomous vehicle mobility](https://trs.jpl.nasa.gov/bitstream/handle/2014/38856/05-3174FN.pdf?isAllowed=y&sequence=1). 

MAVS provides several methods for generating synthetic rough surfaces. In this document, a brief overview of surface roughness will be given, followed by detailed presentations of how MAVS implements rough-surface generation. Finally, an example of how to generate a rough surface with MAVS-Python will be shown.

## Background
Terrain roughness is intuitively understood to be the "bumpiness" of a the terrain surface. With respect to vehicle mobility, the US Army has traditionally defined roughness as the root-mean-squared (RMS) deviation of terrain elevation from the local (detrended) average elevation. In this case, the roughness is [given by](https://www.sciencedirect.com/science/article/pii/S0022489817300642?casa_token=ODINRkZIwLMAAAAA:04L3yQ1Vh_hcTu1zmYL9PCv59L0aBa0CDfmw6smyzEHrgdfn1kciEucweR3AVpEXa5OzBgRK46s)

$\sigma = \sqrt{\frac{\sum_{i=1}^{N}{(z(x_i, y_i)-\overline{z})^2}}{N}}$

where *N* is the total number of elevation measurements, $z(x_i,y_i)$ is the elevation of point *i*, and $\overline{z}$ is the local average elevation. While this definition of roughness is useful for its simplicity, it is not useful for generating synthetic rough terrain or for mathematically evaluating roughness. In fact, surface roughness must be quantified not only by the elevation variance ($\sigma^2$) but also by the length scale of the roughness variation, known as the autocorrelation length, $\beta$. Many previous studies have shown that both the height distribution and length scale can be approximated by a Gaussian function [\[Smith 1967\]](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=1138991&casa_token=iRdVp5paAgoAAAAA:z9NnbI-GeqrMcwfMkIYsN5UTvTHNUQ7lE3njhuHNbgQHaDgMhEtWwKkGv6JyuQZ85HY09WV2vQ&tag=1),[\[Leader 1979\]](https://www.osapublishing.org/view_article.cfm?gotourl=https%3A%2F%2Fwww%2Eosapublishing%2Eorg%2FDirectPDFAccess%2FD1766F1F%2DD313%2D4AAA%2DB27B7C7451A8AE01%5F74672%2Fjosa%2D69%2D4%2D610%2Epdf%3Fda%3D1%26id%3D74672%26shib%3D578006%26seq%3D0%26mobile%3Dno&org=Mississippi%20State%20University),[\[He 1991\]](https://dl.acm.org/doi/pdf/10.1145/127719.122738?casa_token=4C25hCJXKz0AAAAA:qYPAE-rHT8XH3YcGMViGOsgwmG3va2kmUBy99Z3xx_at2VAGoNPzaQBRbMBSxN99pJa2alq0dnvO4w). The autocorrelation function for surfaces is also often modeled as an [exponential function](https://aip.scitation.org/doi/full/10.1063/1.1914947?casa_token=-h33G7WadrAAAAAA%3A1wSyRZufrAhMTJKKcdQe_kcsCl3PtCZAhWYbp0usHgv2X1ZcAIgjvBJf4cXVcBMOhhmyZ8q9Wkjh_Q&). In this case, rough surfaces can be randomly generated using the [Orenstein-Uhlenbeck proces](http://www.numrec.com/whp/fast/OU_process.pdf). This process is implemented in MAVS, discussed in more detail below.

While random Guassian surfaces may be generated with proper roughness statistics, in the last few decades alternative noise models such as [Perlin noise](https://dl.acm.org/doi/pdf/10.1145/325165.325247?casa_token=H5zIHX19thgAAAAA:1o-7-n1CHo9JwwiqHcIWDgtej32zGXZJqsNK_6OrE6FnVVbKoLfoRq8epDP0RvgNeD3WFibnonSSoQ), or more generally, [simplex noise](https://dl.acm.org/doi/pdf/10.1145/566570.566636?casa_token=neFdUJQbGR4AAAAA:0bLiX0nmMXlV5-iwCs83RN6PxQWf33ghoDH0Ns4CE1kq5AVyuL6l4w5ZNcqyzBpFTpaRA1O49dFr3w), have found wide use in computer graphics for their realistic appearance in a variety of phenomenon, from [clouds to soft-tissue to terrain](https://onlinelibrary.wiley.com/doi/pdf/10.1111/j.1467-8659.2010.01827.x?casa_token=JR9h00SSaGoAAAAA:6I2yVSKnn8rvaRHHxBT8e7Xo93JnXlUnMJTdVMztECtfaMCl_aX3o3-PjiQsC0cSGoFs34kIs8OkJvM). Perlin noise is also used in MAVS to generate rough terrain in two different algorithms, as will be discussed in the following sections.

Finally, the most complete characterization of surface roughness is the power-spectral-density (PSD) of the height function of the surface, denoted $P_d$ in the equations below. For terrain roughness, the PSD can be understood as the [relationship between the magnitude of the roughness and the spatial scale (wavelength) of that roughness](https://apps.dtic.mil/sti/pdfs/ADA120945.pdf). 

$\sigma^2 = \int_{0}^{\infty} P_d(\Omega) d\Omega$

where $\sigma$ is the roughness and $\Omega$ is the frequency of the roughness (inverse of the wavelength). The functional form of $P_d$ fully defines the roughness characteristics of a surface. When fitting measurements, the PSD is often modeled as a [power series](https://apps.dtic.mil/sti/pdfs/ADA120945.pdf). Pioneering work by Van Deussen showed that for natural surfaces, the PSD can simply [be modeled as](https://searchworks.stanford.edu/view/8637449)

$P_d(\Omega) = C\Omega^{-2}$

where $\Omega$ has units of ft$^{-1}$. Ideally, synthetically generated rough terrains will also obey this rule. 

## Relevant Spatial Scales for Surface Roughness w.r.t Ground Vehicles
In this section, the relevant length scales for surface roughness are discussed. First, we must consider that we are creating a discretized representation (either in the form of a mesh or heightmap) of a continuous surface. The Nyquist criteria dictates that the shortest wavelength that can be represented by a discretized surface is twice the spatial resolution of the grid/heightmap. Traditionally, the US Army has discretized ride courses at about 1 foot $\approx$ 0.3 meters and filtered out wavelengths over 10 feet $\approx$ 3.0 meters from roughness measurements [\[Durst 2011\]](https://www.sciencedirect.com/science/article/pii/S0022489810000522?casa_token=nXAoJnSuJ-gAAAAA:wKioX6Ll0LwZwZmuJose7lV4U4icFZwvYN8l3f3NbxZrmgFYWwZDHarDn-if9vJFBIeAfoy01EI). While these sizes are most relevant for typical passenger sized military vehicles, they may not be the right choice for smaller unmanned vehicles. Based on typical military vehicle sizes, a good rule of thumb is that wavelengths greater than twice the wheelbase of the vehicle should be filtered out, while wavelengths less than the radius of the tire should be ignored. This would mean that the grid resolution needs to be about half the radius of the tire to meet the Nyquist criteria.

## MAVS Algorithm Overview
MAVS has three different algorithms for generating noise. The first (Algorithm 1) and simplest combines two frequencies of Perlin noise to generate both a rolling hill effect and a surface roughness effect. This option is the "perlin" option in the MAVS-Python API.

The second option (Algorithm 2) for generating surface noise adds high frequency Gaussian noise to low frequency Perlin noise. This is the"gaussian" option in the Mavs-Python API.

For both Algorithm 1 and 2, the high-frequency noise has a wavelength of 2 meters and the low-frequency noise has a wavelength of 50 meters. Therefore, the low-frequeny noise should be filtered out in surface-roughness calculations, while the high-frequency noise will be relevant as surface roughness for most vehicles. 

The final option (Algorithm 3) uses multiple decades of Perlin noise to generate a terrain that follows the PSD for natural terrains measured by [Van Deussen](https://searchworks.stanford.edu/view/8637449), namely

$P_d(\Omega) = C\Omega^{-2}$

In this option, the mesh resolution set by the user defines the upper limit of highest frequency noise, while the mesh size defines the lower limit for the low frequency noise. Multiple decades of noise are generated between the upper and lower limit to create the desired PSD. 

In MAVS, Perlin noise is created using the [FastNoise software library](https://github.com/Auburn/FastNoiseLite/tree/FastNoise-Legacy) while Guassian noise is created using the [normal_distribution function in C++](https://en.cppreference.com/w/cpp/numeric/random/normal_distribution).

### Algorithm 1 - Two frequencies of Perlin Noise
In the first algorithm, two frequencies of Perlin noise are added together. By default, the low frequency has a wavelength of 50 meters, while the high-frequency has a wavelength of 2 meters. The magnitue of each of frequency of noise is defined by the user through the [API](https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/). For reference, the MAVS code is presented below.
```c++
void RandomSurface::GenerateHeightMap(float llx, float lly, float urx, float ury, float res){
  FastNoise hifreq_noise;
  FastNoise lofreq_noise;
  float hi_wl = 1.25f*(float)kPi*hifreq_feature_len_;
  float hi_mag = 4.0f*hifreq_feature_mag_;
  hifreq_noise.SetFrequency(1.0 / hi_wl);
  lofreq_noise.SetFrequency(1.0 / lofreq_feature_len_);

  SetDimensions(llx, lly, urx, ury, res);

  int nx = (int)heightmap_.GetHorizontalDim(); 
  int ny = (int)heightmap_.GetVerticalDim(); 
  for (int i = 0; i < nx; i++) {
    float x = (i + 0.5f)*res;
    for (int j = 0; j < ny; j++) {
      float y = (j + 0.5f)*res;
      float z = (float)(hi_mag * hifreq_noise.GetPerlin(x, y) + lofreq_feature_mag_ * lofreq_noise.GetPerlin(x, y));
      heightmap_.SetHeight(i, j, z);
    }
  }
}
```

### Algorithm 2 - Low-Frequency Perlin + Gaussian Noise
In the second algorithm, high-frequency Gaussian noise is added to low-Frequency Perlin noise. Gaussian noise is generated by the Orenstein-Uhlenbeck process. By default, the low frequency has a wavelength of 50 meters, while the high-frequency has a wavelength of 2 meters. The magnitue of each of frequency of noise is defined by the user through the [API](https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/). For reference, the MAVS code is presented below.
```c++
void RandomSurface::GenerateGaussianHeightMap(float llx, float lly, float urx, float ury, float res) {
  unsigned t = (unsigned)std::time(NULL);
  std::default_random_engine generator(t);
  float rms = hifreq_feature_mag_;
  float acl = hifreq_feature_len_;
  std::normal_distribution<float> distribution(0.0f, rms);
  float lx = urx - llx;
  float ly = ury - lly;
  int nx = (int)(0.5f * lx / acl) + 1; 
  int ny = (int)(0.5f * ly / acl) + 1;
  float res_nyq = std::min(lx / ((float)nx),ly/((float)ny));
  res = std::min(res, res_nyq);
  SetDimensions(llx, lly, urx, ury, res);
  nx = (int)heightmap_.GetHorizontalDim();
  ny = (int)heightmap_.GetVerticalDim();

  //create initial gaussian height field
  std::vector<std::vector<float> > hm = mavs::utils::Allocate2DVector(nx, ny, 0.0f);
  float dx = 2.0f*acl;
  float rm = (float)exp(-dx * acl);
  float sig_m = (float)sqrt(rms*rms*(1.0 - rm * rm));
  hm[0][0] = 0.0f;
  for (int i = 0; i < nx; i++) {
    if (i != 0) {
      std::normal_distribution<float> disti(rm*hm[i-1][0], sig_m);
      hm[i][0] = disti(generator);
    }
    for (int j = 1; j < ny; j++) {
      std::normal_distribution<float> distj(rm*hm[i][j-1], sig_m);
      hm[i][j] = distj(generator);
    }
  }

  // set the heightmap and add low-frequency noise
  FastNoise lofreq_noise;
  lofreq_noise.SetFrequency(1.0 / lofreq_feature_len_);
  for (int i = 0; i < nx; i++) {
    float x = (i + 0.5f)*res;
    for (int j = 0; j < ny; j++) {
      float y = (j + 0.5f)*res;
      float z = hm[i][j]+(float)(lofreq_feature_mag_ * lofreq_noise.GetPerlin(x, y));
      heightmap_.SetHeight(i, j, z);
    }
  }
}
```


### Algorithm 3 - Decades of Perlin Noise
In the final noise algorithm, multiple decades of Perlin noise are added together to create a terrain with the desired PSD. The upper and lower bounds on the frequency are defined by the grid resolution and the overall size of the terrain. In a final pass, the overall roughness is spatially scaled by low-frequency Perlin noise with a wavelength of 100 meters. For reference, the MAVS code is presented below.
```c++
void RandomSurface::GenerateVariableRoughness(float llx, float lly, float urx, float ury, float res){

  FastNoise noise;
  noise.SetFrequency(1.0f);
  FastNoise  rough_scale;
  rough_scale.SetFrequency(0.01f);

  SetDimensions(llx, lly, urx, ury, res); //sets heightmap to 0

  int nx = (int)heightmap_.GetHorizontalDim();
  int ny = (int)heightmap_.GetVerticalDim();

  float max_wl = std::min(100.0f, 0.25f*std::min(urx - llx, ury - lly));
  float min_wl = 2.0f*res;
  int num_decades = 1;
  float wl = min_wl;
  while (wl <= max_wl) {
    wl = (float)pow(2.0, num_decades)*min_wl;
    num_decades = num_decades + 1;
  }
  num_decades--;

  // Populate arrays holding noise coefficients for use later
  std::vector<float> magnitudes; 
  std::vector<float> wavelengths; 
  for (int n=0;n<num_decades;n++) {
    float fac = (float)pow(2.0, n);
    magnitudes.push_back(hifreq_feature_mag_ * fac);
    wavelengths.push_back(min_wl * fac);
  }

  for (int i = 0; i < nx; i++) {
    float x = (i + 0.5f)*res;
    for (int j = 0; j < ny; j++) {
      float y = (j + 0.5f)*res;
      float z = 0.0f;
      for (int n = 0; n < num_decades; n++) {
        float z0 = magnitudes[n] * (float)noise.GetPerlin(x / wavelengths[n], y / wavelengths[n]);
        z = z + z0; 
      }
      float scale_fac = 0.5f*(1.0f + (float)rough_scale.GetPerlin(x, y));
      z = scale_fac * z;
      heightmap_.SetHeight(i, j, z);
    }
  }
}
```

## MAVS Example
For reference, the following code shows how a random surface could be created with the MAVS-Python interface. For more information, see the [API documentation](https://cgoodin.gitlab.io/msu-autonomous-vehicle-simulator/classmavs__python_1_1mavs__interface_1_1_mavs_random_scene.html).
```python
import mavs_interface as mavs

# Create a MAVS Random Scene
random_scene = mavs.MavsRandomScene()

# Define the size of the scene, in meters
random_scene.terrain_width = 100.0 # meters
random_scene.terrain_length = 100.0 # meters

# roughness type can be 'variable', 'gaussian', or 'perlin'
random_scene.surface_roughness_type = "variable"

# When using variable noise
# The lowest frequency of noise will be 2/min(terrain_width,terrain_length)
# The highest frequency of noise will be 1/(2*mesh_resolution)
random_scene.mesh_resolution=0.15

# When surface_roughness_type='variable', this will also generate a file called
# 'rms_truth.txt' that gives the RMS roughness (in meters) on an ENU grid.

# This is the magnitude of low-frequency roughness.
# It is ignored when using the 'variable' noise setting
random_scene.lo_mag = 0.0 # Always set this to 0 when using 'variable' roughness

# Magnitude of the high-frequency roughness
random_scene.hi_mag = 0.1 # The magnitude of the highest frequency noise, should be <0.15 meters

# Set the number of plants in the scene
random_scene.plant_density = 0.05 # 0-1

# Define how you want the trail to look
# Set the trail parameters to zero if you don't want a trail
random_scene.trail_width = 0.0 
random_scene.track_width = 0.0
random_scene.wheelbase = 0.0

# The scene name can be whatever you choos
random_scene.basename = 'rough_surface'

# Choose from the ecosystem files in mavs/data/ecosystems
random_scene.eco_file = 'american_southwest_desert.json'

# Have the trail follow 'Ridges', 'Valleys', or 'Loop'
random_scene.path_type = 'Ridges'

# Create the scene with the properties defined above
random_scene.CreateScene()
```




