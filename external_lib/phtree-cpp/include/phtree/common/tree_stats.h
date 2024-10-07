/*
 * Copyright 2020 Improbable Worlds Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef PHTREE_COMMON_TREE_STATS_H
#define PHTREE_COMMON_TREE_STATS_H

#include "base_types.h"
#include <sstream>
#include <vector>

/*
 * PLEASE do not include this file directly, it is included via common.h.
 *
 * This file defines the type returned by the getStats() method of the PH-Tree.
 * They provide various statistics on the PH-Tree instance that returns them.
 */
namespace improbable::phtree {

class PhTreeStats {
    using SCALAR = scalar_64_t;

  public:
    std::string ToString() {
        std::ostringstream s;
        s << "  nNodes = " << std::to_string(n_nodes_) << std::endl;
        s << "  avgNodeDepth = " << ((double)q_total_depth_ / (double)n_nodes_) << std::endl;
        s << "  AHC=" << n_AHC_ << "  NI=" << n_nt_ << "  nNtNodes_=" << n_nt_nodes_ << std::endl;
        double apl = GetAvgPostlen();
        s << "  avgPostLen = " << apl << " (" << (detail::MAX_BIT_WIDTH<SCALAR> - apl) << ")"
          << std::endl;
        return s.str();
    }

    std::string ToStringHist() {
        std::ostringstream s;
        s << "  infix_len      = ";
        to_string(s, infix_hist_) << std::endl;
        s << "  nodeSizeLog   = ";
        to_string(s, node_size_log_hist_) << std::endl;
        s << "  node_depth_hist_ = ";
        to_string(s, node_depth_hist_) << std::endl;
        s << "  depthHist     = ";
        to_string(s, q_n_post_fix_n_) << std::endl;
        return s.str();
    }

    /*
     * @return average postfix_len, including the HC/LHC bit.
     */
    double GetAvgPostlen() {
        size_t total = 0;
        size_t num_entry = 0;
        for (detail::bit_width_t i = 0; i < detail::MAX_BIT_WIDTH<SCALAR>; ++i) {
            total += (detail::MAX_BIT_WIDTH<SCALAR> - i) * q_n_post_fix_n_[i];
            num_entry += q_n_post_fix_n_[i];
        }
        return (double)total / (double)num_entry;
    }

    size_t GetNodeCount() {
        return n_nodes_;
    }

    size_t GetCalculatedMemSize() {
        return size_;
    }

  private:
    static std::ostringstream& to_string(std::ostringstream& s, std::vector<size_t>& data) {
        s << "[";
        for (size_t x : data) {
            s << x << ",";
        }
        s << "]";
        return s;
    }

  public:
    size_t n_nodes_ = 0;
    size_t n_AHC_ = 0;       // AHC nodes (formerly Nodes with AHC-postfix representation)
    size_t n_nt_nodes_ = 0;  // NtNodes (formerly Nodes with sub-HC representation)
    size_t n_nt_ = 0;        // nodes with NT representation
    size_t n_total_children_ = 0;
    size_t size_ = 0;  // calculated size in bytes
    size_t q_total_depth_ = 0;
    // filled with  x[current_depth] = nPost;
    std::vector<size_t> q_n_post_fix_n_ = std::vector(detail::MAX_BIT_WIDTH<SCALAR>, (size_t)0);
    // prefix len
    std::vector<size_t> infix_hist_ = std::vector(detail::MAX_BIT_WIDTH<SCALAR>, (size_t)0);
    // prefix len
    std::vector<size_t> node_depth_hist_ = std::vector(detail::MAX_BIT_WIDTH<SCALAR>, (size_t)0);
    // log (num_entries)
    std::vector<size_t> node_size_log_hist_ = std::vector(32, (size_t)0);
};

}  // namespace improbable::phtree
#endif  // PHTREE_COMMON_TREE_STATS_H
