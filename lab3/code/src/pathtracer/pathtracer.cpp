#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"

using namespace CGL::SceneObjects;

namespace CGL
{

  PathTracer::PathTracer()
  {
    gridSampler = new UniformGridSampler2D();
    hemisphereSampler = new UniformHemisphereSampler3D();

    tm_gamma = 2.2f;
    tm_level = 1.0f;
    tm_key = 0.18;
    tm_wht = 5.0f;
  }

  PathTracer::~PathTracer()
  {
    delete gridSampler;
    delete hemisphereSampler;
  }

  void PathTracer::set_frame_size(size_t width, size_t height)
  {
    sampleBuffer.resize(width, height);
    sampleCountBuffer.resize(width * height);
  }

  void PathTracer::clear()
  {
    bvh = NULL;
    scene = NULL;
    camera = NULL;
    sampleBuffer.clear();
    sampleCountBuffer.clear();
    sampleBuffer.resize(0, 0);
    sampleCountBuffer.resize(0, 0);
  }

  void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                        size_t y0, size_t x1, size_t y1)
  {
    sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
  }

  Vector3D
  PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                  const Intersection &isect)
  {
    // Estimate the lighting from this intersection coming directly from a light.
    // For this function, sample uniformly in a hemisphere.

    // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
    // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

    // make a coordinate system for a hit point
    // with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    // w_out points towards the source of the ray (e.g.,
    // toward the camera if this is a primary ray)
    // hit_p是在世界坐标系下
    const Vector3D hit_p = r.o + r.d * isect.t;
    const Vector3D w_out = w2o * (-r.d);

    // This is the same number of total samples as
    // estimate_direct_lighting_importance (outside of delta lights). We keep the
    // same number of samples for clarity of comparison.
    int num_samples = scene->lights.size() * ns_area_light;
    Vector3D L_out;

    // TODO (Part 3): Write your sampling loop here
    // TODO BEFORE YOU BEGIN
    // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading
    // 我们将在半球内均匀
    // 采样光线方向。当前只需关心直接照明，因此只需要检查从 hit_p 出发的采样光线是
    // 否与光源相交。一旦估计了入射光的数量，就可以使用反射方程（BRDF）来计算出射
    // 光的比重了。
    // 均匀半球采样的概率密度函数
    double pdf = 1 / (2 * PI);
    for (int i = 0; i < num_samples; i++)
    {
      // object space vector wi,wo
      Vector3D w_in = hemisphereSampler->get_sample();
      // 判断光源是否在采样射线上（min_t设置为一个较小值）
      Ray sample_ray(hit_p, (o2w * w_in).unit());
      sample_ray.min_t = EPS_F;

      Intersection new_isect;
      bool is_isected = bvh->intersect(sample_ray, &new_isect);
      // 采样的射线和包围盒相交了
      if (is_isected)
      {
        // 入射光的radiance (光源的emission)
        Vector3D L_in = new_isect.bsdf->get_emission();
        Vector3D fr = isect.bsdf->f(w_out, w_in);
        // 计算光线方向与法线的余弦（世界坐标下）
        double cos_theta = dot(sample_ray.d, isect.n);

        // 当前方向的光线对出射radiance的贡献（利用反射/渲染方程）
        L_out += Vector3D(L_in.x * fr.x, L_in.y * fr.y, L_in.z * fr.z) * cos_theta / pdf;
      }
    }
    L_out /= num_samples;
    return L_out;
  }

  Vector3D
  PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                  const Intersection &isect)
  {
    // Estimate the lighting from this intersection coming directly from a light.
    // To implement importance sampling, sample only from lights, not uniformly in
    // a hemisphere.

    // make a coordinate system for a hit point
    // with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    // w_out points towards the source of the ray (e.g.,
    // toward the camera if this is a primary ray)
    const Vector3D hit_p = r.o + r.d * isect.t;
    const Vector3D w_out = w2o * (-r.d);
    Vector3D L_out;
  
    // 投射光线、计算求交、计算cos时使用世界坐标系下的采样光线
    // 求物体表面材质（BSDF）时使用对象坐标系

    // 采样所有光源
    int light_num = 0;
    for (auto light: scene->lights)
    {
      light_num ++;
      // 点光源
      if (light->is_delta_light())
      {
        Vector3D w_in;
        double pdf;
        double dist;
        Vector3D L_in = light->sample_L(hit_p, &w_in, &dist, &pdf);
        // 世界坐标系下的w_in
        Ray new_ray(hit_p, w_in);
        new_ray.min_t = EPS_F;
        new_ray.max_t = dist - EPS_F;
        Intersection new_isect;
        Vector3D fr = isect.bsdf->f(w_out, w2o * w_in);
        double cos_theta = dot(w_in, isect.n);
        // 入射光没有被遮挡
        if (!bvh->intersect(new_ray, &new_isect))
        {
          // 计算对出射光的贡献
          L_out += Vector3D(L_in.x * fr.x, L_in.y * fr.y, L_in.z * fr.z) * cos_theta / pdf;
        }
      }
      else
      {
        // 单位面积光源采样数
        for (int i = 0; i < ns_area_light; i++)
        {
          Vector3D w_in;
          double pdf;
          double dist;
          Vector3D L_in = light->sample_L(hit_p, &w_in, &dist, &pdf);
          Ray new_ray(hit_p, w_in);
          new_ray.min_t = EPS_F;
          new_ray.max_t = dist - EPS_F;
          Intersection new_isect;
          Vector3D fr = isect.bsdf->f(w_out, w2o * w_in);
          double cos_theta = dot(w_in, isect.n);
          // 入射光没有被遮挡
          if (!bvh->intersect(new_ray, &new_isect))
          {
            // 计算对出射光的贡献
            L_out += Vector3D(L_in.x * fr.x, L_in.y * fr.y, L_in.z * fr.z) * cos_theta / pdf;
          }
        }
        // 对单个面光源采样数归一化
        L_out /= ns_area_light;
      }
    }
    return L_out;
  }

  Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                            const Intersection &isect)
  {
    // TODO: Part 3, Task 2
    // Returns the light that results from no bounces of light
    return isect.bsdf->get_emission();
  }

  Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                           const Intersection &isect)
  {
    // TODO: Part 3, Task 3
    // Returns either the direct illumination by hemisphere or importance sampling
    // depending on `direct_hemisphere_sample`
    direct_hemisphere_sample = false;
    if (direct_hemisphere_sample)
    {
      return estimate_direct_lighting_hemisphere(r, isect);
    }
    return estimate_direct_lighting_importance(r, isect);
  }

  Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                    const Intersection &isect)
  {
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    Vector3D hit_p = r.o + r.d * isect.t;
    Vector3D w_out = w2o * (-r.d);

    Vector3D L_out(0, 0, 0);

    // TODO: Part 4, Task 2
    // Returns the one bounce radiance + radiance from extra bounces at this point.
    // Should be called recursively to simulate extra bounces.

    return L_out;
  }

  Vector3D PathTracer::est_radiance_global_illumination(const Ray &r)
  {
    Intersection isect;
    Vector3D L_out;

    // You will extend this in assignment 3-2.
    // If no intersection occurs, we simply return black.
    // This changes if you implement hemispherical lighting for extra credit.

    // The following line of code returns a debug color depending
    // on whether ray intersection with triangles or spheres has
    // been implemented.
    //
    // REMOVE THIS LINE when you are ready to begin Part 3.

    if (!bvh->intersect(r, &isect))
      return envLight ? envLight->sample_dir(r) : L_out;

    L_out = (isect.t == INF_D) ? debug_shading(r.d) : zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);

    // TODO (Part 3): Return the direct illumination.
    // TODO (Part 4): Accumulate the "direct" and "indirect"
    // parts of global illumination into L_out rather than just direct

    return L_out;
  }

  void PathTracer::raytrace_pixel(size_t x, size_t y)
  {
    // TODO (Part 1.2):
    // Make a loop that generates num_samples camera rays and traces them
    // through the scene. Return the average Vector3D.
    // You should call est_radiance_global_illumination in this function.

    // TODO (Part 5):
    // Modify your implementation to include adaptive sampling.
    // Use the command line parameters "samplesPerBatch" and "maxTolerance"

    Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel
    int num_samples = ns_aa;          // total samples to evaluate
    Vector3D total_radiance = Vector3D(0, 0, 0);
    for (int i = 0; i < num_samples; i++)
    {
      // 在方形像素上随机均匀采样
      Vector2D random = gridSampler->get_sample();
      random.x += origin.x;
      random.y += origin.y;
      // 该路径上光线的radiance
      Ray r = camera->generate_ray(random.x / sampleBuffer.w, random.y / sampleBuffer.h);
      total_radiance += est_radiance_global_illumination(r);
    }

    // 返回平均radiance
    sampleBuffer.update_pixel(total_radiance / num_samples, x, y);
    sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
  }

  void PathTracer::autofocus(Vector2D loc)
  {
    Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
    Intersection isect;

    bvh->intersect(r, &isect);

    camera->focalDistance = isect.t;
  }

} // namespace CGL
