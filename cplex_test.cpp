#include <iostream>
#include <ilcplex/ilocplex.h>   // Concert Technology header

ILOSTLBEGIN   // Required macro for CPLEX + STL compatibility on Windows

int main() {
    IloEnv env;   // CPLEX environment (must be created first)

    try {
        // ── 1. Create model ───────────────────────────────────────────────
        IloModel model(env);

        // ── 2. Define decision variables ──────────────────────────────────
        // IloNumVar(env, lower_bound, upper_bound, type)
        IloNumVar x(env, 0.0, IloInfinity, ILOFLOAT);   // x >= 0
        IloNumVar y(env, 0.0, IloInfinity, ILOFLOAT);   // y >= 0

        // Give variables names (helpful for debugging)
        x.setName("x");
        y.setName("y");

        // ── 3. Set objective: Maximize x + 2y ─────────────────────────────
        model.add(IloMaximize(env, x + 2 * y));

        // ── 4. Add constraints ────────────────────────────────────────────
        model.add(x + y <= 10);   // c1
        model.add(x      <= 6);   // c2
        model.add(y      <= 8);   // c3

        // ── 5. Create solver and attach model ─────────────────────────────
        IloCplex cplex(model);

        // Suppress CPLEX output (set to 0 to hide solver log)
        cplex.setParam(IloCplex::Param::MIP::Display, 0);
        cplex.setParam(IloCplex::Param::Simplex::Display, 0);

        // ── 6. Solve ──────────────────────────────────────────────────────
        if (!cplex.solve()) {
            std::cerr << "ERROR: Failed to solve the model." << std::endl;
            env.end();
            return 1;
        }

        // ── 7. Print results ──────────────────────────────────────────────
        std::cout << "===== CPLEX Test: LP Solution =====" << std::endl;
        std::cout << "Status     : " << cplex.getStatus()          << std::endl;
        std::cout << "Objective  : " << cplex.getObjValue()        << std::endl;
        std::cout << "x          : " << cplex.getValue(x)         << std::endl;
        std::cout << "y          : " << cplex.getValue(y)         << std::endl;
        std::cout << "===================================" << std::endl;

        // ── 8. Verify expected result ─────────────────────────────────────
        double expectedObj = 18.0;
        if (std::abs(cplex.getObjValue() - expectedObj) < 1e-6) {
            std::cout << "PASS: Objective matches expected value of " 
                      << expectedObj << std::endl;
        } else {
            std::cout << "FAIL: Expected " << expectedObj 
                      << " but got " << cplex.getObjValue() << std::endl;
        }

    } catch (IloException& e) {
        // CPLEX-specific exceptions
        std::cerr << "CPLEX Exception: " << e.getMessage() << std::endl;
        env.end();
        return 1;
    } catch (std::exception& e) {
        std::cerr << "Standard Exception: " << e.what() << std::endl;
        env.end();
        return 1;
    }

    env.end();   // Always call env.end() to free CPLEX memory
    return 0;
}
