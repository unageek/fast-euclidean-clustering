# 🚀 fast-euclidean-clustering ✨

Fast Euclidean clustering (FEC) of point clouds implemented for PCL. No GPU is required!

FEC is an approximation algorithm. A cluster may be falsely splitted into multiple clusters, but not the other way around.

## How to Use

Just copy [`fast_euclidean_clustering.h`](fast_euclidean_clustering.h) to local and `#include` it!

## Benchmark

**#points = 1,000,071**

<table>
  <thead>
    <tr>
      <th rowspan="2">Tolerance (m)
      <th colspan="2">FEC (q = 0.0)
      <th colspan="2">FEC (q = 0.5)
      <th colspan="2">FEC (q = 0.9)
      <th colspan="2">EC
    <tr>
      <th>#clusters
      <th>Time (s)
      <th>#clusters
      <th>Time (s)
      <th>#clusters
      <th>Time (s)
      <th>#clusters
      <th>Time (s)
  <tbody>
    <tr>
      <td align="right">0.01
      <td align="right">464,420
      <td align="right">0.9
      <td align="right">442,669
      <td align="right">0.9
      <td align="right">441,151
      <td align="right">1.0
      <td align="right">441,151
      <td align="right">1.0
    <tr>
      <td align="right">0.1
      <td align="right">22,592
      <td align="right">0.4
      <td align="right">20,422
      <td align="right">0.7
      <td align="right">20,261
      <td align="right">4.5
      <td align="right">20,261
      <td align="right">8.9
    <tr>
      <td align="right">1
      <td align="right">175
      <td align="right">0.3
      <td align="right">163
      <td align="right">0.8
      <td align="right">160
      <td align="right">15.0
      <td align="right">160
      <td align="right">795.8
    <tr>
      <td align="right">10
      <td align="right">37
      <td align="right">0.3
      <td align="right">35
      <td align="right">0.7
      <td align="right">33
      <td align="right">11.0
      <td align="right">N/A¹
      <td align="right">&gt;1,000¹
    <tr>
      <td align="right">100
      <td align="right">2
      <td align="right">0.3
      <td align="right">2
      <td align="right">0.5
      <td align="right">2
      <td align="right">2.7
      <td align="right">N/A¹
      <td align="right">&gt;1,000¹
</table>

¹ Computation did not complete within 1,000 seconds.

- FEC: `FastEuclideanClustering`, q is the quality parameter.
- EC: `pcl::EuclideanClusterExtraction`.

## Example

See [`examples/fec/main.cc`](examples/fec/main.cc).
