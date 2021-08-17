#ifndef GRPOSE_UTIL_TYPE_TRAITS_
#define GRPOSE_UTIL_TYPE_TRAITS_

#include <type_traits>

namespace grpose {

template <typename EigenType>
constexpr bool kIsStatic =
    (std::remove_reference_t<EigenType>::SizeAtCompileTime != Eigen::Dynamic);

template <typename EigenType>
constexpr bool kIsVector2Static =
    (std::is_same_v<typename std::remove_reference_t<EigenType>::Scalar,
                    double> &&
     std::remove_reference_t<EigenType>::RowsAtCompileTime == 2 &&
     std::remove_reference_t<EigenType>::ColsAtCompileTime == 1);

template <typename EigenType>
constexpr bool kIsVector3Static =
    (std::is_same_v<typename std::remove_reference_t<EigenType>::Scalar,
                    double> &&
     std::remove_reference_t<EigenType>::RowsAtCompileTime == 3 &&
     std::remove_reference_t<EigenType>::ColsAtCompileTime == 1);

template <typename EigenType>
inline bool IsVector2Dynamic(const Eigen::MatrixBase<EigenType> &object);

template <typename EigenType>
inline bool IsVector3Dynamic(const Eigen::MatrixBase<EigenType> &object);

// Implementation

template <typename EigenType>
inline bool IsVector2Dynamic(const Eigen::MatrixBase<EigenType> &object) {
  return std::is_same_v<EigenType, double> && object.rows() == 2 &&
         object.cols() == 1;
}

template <typename EigenType>
inline bool IsVector3Dynamic(const Eigen::MatrixBase<EigenType> &object) {
  return std::is_same_v<typename EigenType::Scalar, double> &&
         object.rows() == 3 && object.cols() == 1;
}

}  // namespace grpose

#define GRPOSE_CHECK_IS_VECTOR2(object)                              \
  do {                                                               \
    if constexpr (kIsStatic<decltype((object))>)                     \
      static_assert(kIsVector2Static<decltype((object))>,            \
                    "Expected Vector2-sized object!");               \
    else if (!IsVector2Dynamic((object)))                            \
      throw std::invalid_argument("Expected Vector2-sized object!"); \
  } while (false)

#define GRPOSE_CHECK_IS_VECTOR3(object)                              \
  do {                                                               \
    if constexpr (kIsStatic<decltype((object))>)                     \
      static_assert(kIsVector3Static<decltype((object))>,            \
                    "Expected Vector3-sized object!");               \
    else if (!IsVector3Dynamic((object)))                            \
      throw std::invalid_argument("Expected Vector3-sized object!"); \
  } while (false)
#endif