# 作业一

## 作业

### 光栅化画三角形

经过坐标变化和的三角形x y 已经在像素坐标下 我们可以通过叉积来判断点在三角形内
这里有个细节主要要讨论三角形点连接的顺序是顺时针还是逆时针 链接顺序关系到叉积判断
点在三角形内或外

关于叉积公式

$\vec a \times \vec b = |\vec a||\vec b| \sin \theta \vec n$

其中
+ $\theta$是夹角(注意角度有方向 方向是$\vec a$到$\vec b$  角度可能大于180)

+ $\vec n$是单位向量,方向垂直于$\vec a ，\vec b$ 且遵循右手定则

![](https://data-saber.oss-cn-guangzhou.aliyuncs.com/known/15314632595598.png)

```cpp
static bool insideTriangle(int x, int y, const Vector3f* _v)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    // CW

    auto v1 = Vector3f(x, y, 0);

    auto v2 = Vector3f(_v->x(), _v->y(), 0);

    auto val = v1.cross(v2);

    // std::cout << "v1 : " << v1 << std::endl;

    // std::cout << "v2 : " << v2 << std::endl;

    // std::cout << "val : " << val << std::endl;

    return val.z() <  0;
}
```

### MSAA超采样

我们可以通过采样$2 \times 2$范围的像素通过判断在三角形内的数量来对颜色进行加权平均

```cpp

void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    Vector3f line_a(t.v[1] - t.v[0]);
    Vector3f line_b(t.v[2] - t.v[1]);
    Vector3f line_c(t.v[0] - t.v[2]);

    int dx[] = {0, 0, 1, 1};
    int dy[] = {0, 1, 0, 1};
    uint32_t n_inside = 0;
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {

         // bool is_inside_a = insideTriangle(j - t.v[0].x(), i - t.v[0].y(), &line_a);
         // bool is_inside_b = insideTriangle(j - t.v[1].x(), i - t.v[1].y(), &line_b);
         // bool is_inside_c = insideTriangle(j - t.v[2].x(), i - t.v[2].y(), &line_c);

        int count = 0;

         for (int k = 0; k < 4; k++) {
           int sx = j + dx[k];
           int sy = i + dy[k];

           bool is_inside_a = insideTriangle(sx - t.v[0].x(), sy - t.v[0].y(), &line_a);
           bool is_inside_b = insideTriangle(sx - t.v[1].x(), sy - t.v[1].y(), &line_b);
           bool is_inside_c = insideTriangle(sx - t.v[2].x(), sy - t.v[2].y(), &line_c);
           if((is_inside_a && is_inside_b && is_inside_c) || (!is_inside_a && !is_inside_b && !is_inside_c)) {

              auto[alpha, beta, gamma] = computeBarycentric2D(j, i, t.v);
              float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
              float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
              z_interpolated *= w_reciprocal;
              count++;
           }
         }

         if(count) {

            auto[alpha, beta, gamma] = computeBarycentric2D(j, i, t.v);
            float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;

            if (z_interpolated <= depth_buf[get_index(j, i)]) {
               depth_buf[get_index(j, i)] = z_interpolated;
               auto col = t.getColor() * count / 4.0;
               set_pixel(Vector3f(j, i, 0), col);
            }
         }
      }
    }


    std::cout << "width : " << width << " height : " << height << " n_insize : " << n_inside << std::endl;
}
```

## 参考

+ [GAMES101作业2](https://www.cnblogs.com/zzysmemory/p/14655679.html)

+ [从点积和叉积开始](https://zhuanlan.zhihu.com/p/481951051)
