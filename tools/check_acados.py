from __future__ import annotations

import os
import shutil
import statistics
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
ACADOS_ROOT = PROJECT_ROOT / "vendor" / "acados"
ACADOS_LIB_PATH = ACADOS_ROOT / "lib"
ACADOS_DLL_PATH = ACADOS_ROOT / "bin"
EXPORT_DIR = PROJECT_ROOT / "artifacts" / "acados_smoke"


def configure_runtime() -> None:
    os.environ["ACADOS_SOURCE_DIR"] = str(ACADOS_ROOT)
    os.environ["ACADOS_LIB_PATH"] = str(ACADOS_LIB_PATH)
    interface_path = (
        ACADOS_ROOT / "interfaces" / "acados_template"
    )
    sys.path.insert(0, str(interface_path))
    if os.name == "nt":
        os.add_dll_directory(str(ACADOS_DLL_PATH))
        gcc_path = shutil.which("gcc")
        if gcc_path is not None:
            compiler_bin = str(Path(gcc_path).resolve().parent)
            os.add_dll_directory(compiler_bin)
            os.environ["PATH"] = (
                compiler_bin + os.pathsep + os.environ.get("PATH", "")
            )


def build_solver(verbose: bool = True):
    import casadi as ca
    from acados_template import (
        AcadosOcp,
        AcadosOcpSolver,
        ocp_get_default_cmake_builder,
    )

    ocp = AcadosOcp()
    ocp.model.name = "acados_smoke"
    position = ca.SX.sym("position")
    velocity = ca.SX.sym("velocity")
    acceleration = ca.SX.sym("acceleration")
    x = ca.vertcat(position, velocity)
    u = ca.vertcat(acceleration)
    ocp.model.x = x
    ocp.model.u = u
    ocp.model.f_expl_expr = ca.vertcat(velocity, acceleration)

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr = ca.vertcat(x, u)
    ocp.model.cost_y_expr_e = x
    ocp.cost.W = ca.diagcat(
        ca.diag([10.0, 10.0]),
        ca.diag([0.1]),
    ).full()
    ocp.cost.W_e = ca.diag([100.0, 100.0]).full()
    ocp.cost.yref = [1.0, 0.0, 0.0]
    ocp.cost.yref_e = [1.0, 0.0]
    ocp.cost.W_0 = ocp.cost.W
    ocp.cost.yref_0 = ocp.cost.yref

    ocp.constraints.x0 = [0.0, 0.0]
    ocp.constraints.lbu = [-2.0]
    ocp.constraints.ubu = [2.0]
    ocp.constraints.idxbu = [0]

    ocp.solver_options.N_horizon = 20
    ocp.solver_options.tf = 0.4
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.print_level = 0
    ocp.code_gen_options.code_export_directory = str(EXPORT_DIR)

    builder = ocp_get_default_cmake_builder()
    if os.name == "nt":
        builder.generator = "MinGW Makefiles"
        builder.additional_cmake_options = (
            "-DCMAKE_C_COMPILER=gcc "
            "-DCMAKE_CXX_COMPILER=g++ "
            "-DCMAKE_SHARED_LIBRARY_PREFIX="
        )
    builder.build_dir = str(EXPORT_DIR / "build")
    return AcadosOcpSolver(
        ocp,
        cmake_builder=builder,
        verbose=verbose,
    )


def main() -> int:
    configure_runtime()
    solver = build_solver()
    status = solver.solve()
    if status != 0:
        raise RuntimeError(f"acados smoke solve failed: status={status}")

    solve_times_ms: list[float] = []
    for _ in range(100):
        started = time.perf_counter()
        status = solver.solve()
        solve_times_ms.append((time.perf_counter() - started) * 1000.0)
        if status != 0:
            raise RuntimeError(
                f"acados smoke solve failed: status={status}"
            )

    ordered = sorted(solve_times_ms)
    print(
        "status=ok "
        f"mean_ms={statistics.mean(solve_times_ms):.4f} "
        f"p50_ms={statistics.median(solve_times_ms):.4f} "
        f"p95_ms={ordered[int(0.95 * (len(ordered) - 1))]:.4f} "
        f"max_ms={max(solve_times_ms):.4f} "
        f"x={solver.get(0, 'x')} u={solver.get(0, 'u')}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
