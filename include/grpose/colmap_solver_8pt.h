#ifndef GRPOSE_COLMAP_SOLVER_8PT_
#define GRPOSE_COLMAP_SOLVER_8PT_

#include "grpose/bearing_vector_correspondences.h"
#include "minimal_solver.h"
#include "util/types.h"

namespace grpose {

    class ColmapSolver8pt : public MinimalSolver {
    public:
        ColmapSolver8pt(
                const std::shared_ptr<BearingVectorCorrespondences> &correspondences,
                const StdVectorA<SE3> &body_from_camera);

        int MinSampleSize() const override;

        bool SolveTimed(const std::vector<int> &correspondence_indices,
                        StdVectorA<SE3> &frame1_from_frame2,
                        double &time_in_seconds) const override;

    private:
        std::shared_ptr<BearingVectorCorrespondences> correspondences_;
        StdVectorA<SE3> body_from_camera_;
    };

}  // namespace grpose


#endif //GRPOSE_COLMAP_SOLVER_8PT_
