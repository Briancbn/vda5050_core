/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <pybind11/pybind11.h>
#ifdef PYBIND11_HAS_INTERNALS_WITH_SMART_HOLDER_SUPPORT
#error "pybind11 >= 3.0 is not supported. Please use pybind11 < 3.0."
#endif

#include "vda5050_core_py/client.hpp"
#include "vda5050_core_py/master.hpp"
#include "vda5050_core_py/rmf_migration.hpp"
#include "vda5050_core_py/transport.hpp"
#include "vda5050_core_py/types.hpp"

PYBIND11_MODULE(_core, m)
{
  m.doc() = "VDA5050 Core Python bindings";

  vda5050_core_py::bind_types(m);
  vda5050_core_py::bind_client(m);
  vda5050_core_py::bind_transport(m);
  vda5050_core_py::bind_master(m);
  vda5050_core_py::bind_rmf_migration(m);
}
