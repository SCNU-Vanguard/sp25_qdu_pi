// [Pi灯条] 可手算的真实BGR像素+生产修正器；不将合成图结果称为HEF/实机识别通过。
#include "tasks/auto_aim/lightbar_refiner.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
using namespace auto_aim;
void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}
template <typename F> void rejects(F function)
{
  try { function(); } catch (const std::invalid_argument &) { return; }
  throw std::runtime_error("Bad refiner configuration was accepted");
}
cv::Mat blank() { return cv::Mat(540, 720, CV_8UC3, cv::Scalar(10, 10, 10)); }
void light(cv::Mat & image, int x, int y0 = 180, int y1 = 300,
           cv::Scalar color = {255, 220, 40})
{
  cv::rectangle(image, {x - 6, y0}, {x + 6, y1}, color, -1);
}
Armor candidate(float x0, float y0, float x1, float y1, double confidence = .98)
{
  return Armor(blue, three, big, confidence,
    cv::Rect(static_cast<int>(std::floor(x0)), static_cast<int>(std::floor(y0)),
      static_cast<int>(std::ceil(x1) - std::floor(x0)),
      static_cast<int>(std::ceil(y1) - std::floor(y0))),
    {{x0,y0}, {x1,y0}, {x1,y1}, {x0,y1}}, {720,540});
}
std::list<Armor> duplicates()
{
  return {candidate(190,230,470,310), candidate(195,210,475,295,.94),
          candidate(197,178,480,308,.91)};
}
void check_points(const Armor & armor, const std::vector<cv::Point2f> & expected, double tolerance)
{
  require(armor.points.size() == expected.size(), "Wrong point count");
  for (std::size_t i = 0; i < expected.size(); ++i)
    require(cv::norm(armor.points[i] - expected[i]) <= tolerance, "Wrong physical lightbar endpoint");
}
// [Pi灯条] 仅测试产物的可视化，不引入Linux HTTP依赖，不参与检测实现。
cv::Mat drawing(const cv::Mat & image, const std::list<Armor> & armors, const char * caption)
{
  cv::Mat result = image.clone();
  cv::putText(result,caption,{15,30},cv::FONT_HERSHEY_SIMPLEX,.6,{255,255,255},1);
  for (const auto & armor : armors) for (int i=0;i<4;++i) {
    cv::line(result,armor.points[i],armor.points[(i+1)%4],{0,255,0},2);
    cv::circle(result,armor.points[i],3,{0,255,255},-1);
  }
  return result;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    LightbarRefineOptions options;
    options.enabled = true;
    LightbarRefiner refiner(options);
    cv::Mat padded(540, 724, CV_8UC3, cv::Scalar(10,10,10));
    auto image = padded.colRange(2, 722);  // [Pi灯条] 非连续Mat与真实相机ROI调用一致。
    light(image, 200); light(image, 480);
    const auto original = padded.clone();
    auto input = duplicates();
    input.front().priority = first;
    input.front().class_id = 12;
    const auto output = refiner.refine(image, input);
    require(output.size() == 1, "Three candidates on same lightbar pair were not merged");
    const auto & armor = output.front();
    check_points(armor, {{200,180},{480,180},{480,300},{200,300}}, 0.01);
    require(armor.center == cv::Point2f(340,240) && armor.box == cv::Rect(200,180,280,120),
            "Center/bbox were left stale after point refinement");
    require(cv::norm(armor.center_norm - cv::Point2f(340.F/720,240.F/540)) < 1e-6 &&
            std::abs(armor.ratio - 280.0/120) < 1e-6 && armor.type == small,
            "Normalized center/ratio/type were not rebuilt");
    require(armor.color == blue && armor.name == three && armor.confidence == .98 &&
            armor.priority == first && armor.class_id == 12 && !armor.duplicated,
            "Candidate semantics changed or lower-confidence duplicate selected");
    require(armor.left.id != armor.right.id && armor.left.top == armor.points[0] &&
            armor.right.bottom == armor.points[2], "Physical lightbar fields inconsistent");
    const auto stats = refiner.stats();
    require(stats.input == 3 && stats.lights == 2 && stats.duplicates == 2 && stats.output == 1 &&
            stats.unmatched == 0 && stats.ambiguous == 0, "Wrong refinement accounting");
    require(cv::norm(padded, original, cv::NORM_INF) == 0 && input.front().points[0] == cv::Point2f(190,230),
            "Refinement changed source image or caller-owned candidates");

    // [Pi灯条] 可选生成明显标注为合成的前后对照，供检查绘图；不编辑用户截图。
    if (argc == 2) {
      const std::filesystem::path directory(argv[1]);
      std::filesystem::create_directories(directory);
      require(cv::imwrite((directory / "synthetic-before.png").string(), drawing(image, input, "SYNTHETIC: before, 3 candidates")) &&
              cv::imwrite((directory / "synthetic-after.png").string(), drawing(image, output, "SYNTHETIC: after, 1 corrected armor")),
              "Cannot write synthetic diagnostic drawings");
    }
    LightbarRefiner disabled;
    const auto unchanged = disabled.refine(image, input);
    require(unchanged.size() == 3 && unchanged.front().points == input.front().points &&
            unchanged.front().type == big, "Default-off changed baseline output");

    // [Pi灯条] 两块真实装甲必须保留，不能把“一个框”实现为全画面强制取最高分。
    auto two = blank();
    for (int x : {100,220,440,560}) light(two, x);
    auto pair = refiner.refine(two, {candidate(100,180,220,300), candidate(440,180,560,300,.9)});
    require(pair.size() == 2, "Two distinct real armors were merged");
    auto moved = blank(); light(moved, 210); light(moved, 490);
    require(refiner.refine(moved, {candidate(200,230,480,310)}).size() == 1, "Moving plate lost");
    check_points(output.front(), {{200,180},{480,180},{480,300},{200,300}}, 0.01);

    // [Pi灯条] 对齐原SP25：无可靠灯条只跳过修正，不能让网络已有候选凭空消失。
    auto no_lights = blank();
    const auto no_light_result = refiner.refine(no_lights, input);
    require(no_light_result.size() == 3 && refiner.stats().unmatched == 3 &&
            refiner.stats().fallback == 3 && no_light_result.front().points == input.front().points,
            "No-light frame dropped instead of retaining the network candidate");
    light(no_lights, 200);
    require(refiner.refine(no_lights, input).size() == 3 && refiner.stats().fallback == 3,
            "Single lightbar did not retain the network candidate");
    auto white = blank(); light(white,200,180,300,{255,255,255}); light(white,480,180,300,{255,255,255});
    require(refiner.refine(white,input).size() == 3 && refiner.stats().fallback == 3,
            "White reflections did not fall back to the network candidate");
    auto red_image = blank(); light(red_image,200,180,300,{40,220,255}); light(red_image,480,180,300,{40,220,255});
    require(refiner.refine(red_image,input).size() == 3 && refiner.stats().fallback == 3,
            "Opposite-color lights did not fall back to the network candidate");
    auto red_candidate = candidate(190,230,470,310); red_candidate.color = red;
    require(refiner.refine(red_image,{red_candidate}).size() == 1, "Red lightbars unsupported");
    // [Pi灯条·实拍] 红/蓝单通道已亮、灰度仍低于150时，不能把灯条上端截掉。
    auto saturated_red = blank();
    light(saturated_red,200,180,300,{20,40,255});
    light(saturated_red,480,180,300,{20,40,255});
    const auto red_result = refiner.refine(saturated_red,{red_candidate});
    require(red_result.size() == 1 && refiner.stats().fallback == 0,
            "Saturated red lamps did not refine");
    check_points(red_result.front(),{{200,180},{480,180},{480,300},{200,300}},0.01);
    auto saturated_blue = blank();
    light(saturated_blue,200,180,300,{255,40,20});
    light(saturated_blue,480,180,300,{255,40,20});
    const auto blue_result = refiner.refine(saturated_blue,{candidate(190,230,470,310)});
    require(blue_result.size() == 1 && refiner.stats().fallback == 0,
            "Saturated blue lamps did not refine");
    check_points(blue_result.front(),{{200,180},{480,180},{480,300},{200,300}},0.01);
    auto single = blank(); light(single,200);
    require(refiner.refine(single,{candidate(180,180,220,300)}).size() == 1 &&
            refiner.stats().fallback == 1, "One-light frame did not fall back");
    auto ambiguous = blank(); light(ambiguous,200); light(ambiguous,230); light(ambiguous,480);
    require(refiner.refine(ambiguous,{candidate(215,180,480,300)}).size() == 1 &&
            refiner.stats().ambiguous == 1 && refiner.stats().fallback == 1,
            "Ambiguous neighboring light pair did not fall back");
    auto edge = blank(); light(edge,6); light(edge,286);
    require(refiner.refine(edge,{candidate(6,180,286,300)}).size() == 1 &&
            refiner.stats().fallback == 1, "Truncated border light did not fall back");
    auto uneven = blank(); light(uneven,200); light(uneven,480,230,275);
    require(refiner.refine(uneven,input).size() == 3 && refiner.stats().fallback == 3,
            "Strongly mismatched light lengths did not fall back");
    require(refiner.refine(image,{candidate(200,50,480,100)}).size() == 1 &&
            refiner.stats().fallback == 1, "Remote lights did not fall back");
    require(refiner.refine(image,{}).empty() && refiner.stats().lights == 0, "Empty frame reused old candidates");

    // [Pi灯条] 倾斜板保留LT/RT/RB/LB语义，不强行把四点变成水平矩形。
    const auto transform = cv::getRotationMatrix2D({340,240}, 15, 1);
    cv::Mat tilted;
    cv::warpAffine(image, tilted, transform, image.size(), cv::INTER_NEAREST);
    std::vector<cv::Point2f> expected{{200,180},{480,180},{480,300},{200,300}}, predicted;
    cv::transform(expected, predicted, transform);
    const auto rect = cv::boundingRect(predicted);
    const Armor tilted_candidate(blue,three,small,.98,rect,predicted,image.size());
    const auto tilted_result = refiner.refine(tilted,{tilted_candidate});
    require(tilted_result.size() == 1, "Tilted lightbars rejected");
    check_points(tilted_result.front(),predicted,2.0);

    for (int invalid : {0,255}) {
      auto bad = options; bad.threshold = invalid;
      rejects([&] { LightbarRefiner rejected(bad); });
    }
    for (double invalid : {0.0,1.1,std::numeric_limits<double>::quiet_NaN()}) {
      auto bad = options; bad.max_corner_shift_ratio = invalid;
      rejects([&] { LightbarRefiner rejected(bad); });
    }
    rejects([&] { refiner.refine(cv::Mat(),input); });
    std::cout << "PASS lightbar refinement: endpoint accuracy, geometry/type rebuild, same-pair duplicates, "
                 "two real armors, color, ambiguity, no lights, border, motion/lifetime, tilted board, default off, config\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return 1;
  }
}
