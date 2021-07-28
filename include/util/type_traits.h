#ifndef GRPOSE_UTIL_TYPE_TRAITS_
#define GRPOSE_UTIL_TYPE_TRAITS_

namespace grpose {

template <typename EigenType>
constexpr bool IsStatic(const Eigen::MatrixBase<EigenType> &object);

template <typename EigenType>
constexpr bool IsVector2Static(const Eigen::MatrixBase<EigenType> &object);

template <typename EigenType>
inline bool IsVector2Dynamic(const Eigen::MatrixBase<EigenType> &object);

template <typename EigenType>
constexpr bool IsVector3Static(const Eigen::MatrixBase<EigenType> &object);

template <typename EigenType>
inline bool IsVector3Dynamic(const Eigen::MatrixBase<EigenType> &object);

// Implementation

template <typename EigenType>
inline bool IsVector3Dynamic(const Eigen::MatrixBase<EigenType> &object);

template <typename EigenType>
constexpr bool IsStatic(const Eigen::MatrixBase<EigenType> &object) {
  return EigenType::SizeAtCompileTime != Eigen::Dynamic;
}

template <typename EigenType>
constexpr bool IsVector2Static(const Eigen::MatrixBase<EigenType> &object) {
  return std::is_same_v<typename EigenType::Scalar, double> &&
         EigenType::RowsAtCompileTime == 2 && EigenType::ColsAtCompileTime == 1;
}

template <typename EigenType>
inline bool IsVector2Dynamic(const Eigen::MatrixBase<EigenType> &object) {
  return std::is_same_v<EigenType, double> && object.rows() == 2 &&
         object.cols() == 1;
}

template <typename EigenType>
constexpr bool IsVector3Static(const Eigen::MatrixBase<EigenType> &object) {
  return std::is_same_v<typename EigenType::Scalar, double> &&
         EigenType::RowsAtCompileTime == 3 && EigenType::ColsAtCompileTime == 1;
}

template <typename EigenType>
inline bool IsVector3Dynamic(const Eigen::MatrixBase<EigenType> &object) {
  return std::is_same_v<typename EigenType::Scalar, double> &&
         object.rows() == 3 && object.cols() == 1;
}

}  // namespace grpose

#define GRPOSE_CHECK_IS_VECTOR2(object)                              \
  {                                                                  \
    if constexpr (IsStatic((object)))                                \
      static_assert(IsVector2Static((object)),                       \
                    "Expected Vector2-sized object!");               \
    else if (!IsVector2Dynamic((object)))                            \
      throw std::invalid_argument("Expected Vector2-sized object!"); \
  }

#define GRPOSE_CHECK_IS_VECTOR3(object)                              \
  {                                                                  \
    if constexpr (IsStatic((object)))                                \
      static_assert(IsVector3Static((object)),                       \
                    "Expected Vector3-sized object!");               \
    else if (!IsVector3Dynamic((object)))                            \
      throw std::invalid_argument("Expected Vector3-sized object!"); \
  }
#endif