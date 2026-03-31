/*
 * Ocam Projection Verification
 * Compare our implementation with CPAC reference
 */
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <json/json.h>

using namespace std;

// CPAC-style Ocam projection
void point2pixel_cpac(double x, double y, double z,
                      const vector<double>& invpol,
                      double c, double d, double e,
                      double cx, double cy,
                      double& u, double& v) {
    double norm = sqrt(x*x + y*y);
    if (norm < 1e-14) norm = 1e-14;
    
    // CPAC: theta = atan(-z / norm)
    double theta = atan(-z / norm);
    
    // CPAC: Horner's method, coefficients in original order
    double rho = 0.0;
    for (int i = invpol.size() - 1; i >= 0; i--) {
        rho = rho * theta + invpol[i];
    }
    
    double uu = x / norm * rho;
    double vv = y / norm * rho;
    
    // CPAC affine: u = uu + vv*e + cx, v = uu*d + vv*c + cy
    u = uu + vv * e + cx;
    v = uu * d + vv * c + cy;
}

// Our current implementation
void point2pixel_our(double x, double y, double z,
                     const std::vector<double>& world2cam_coeffs,  // Already reversed
                     double c, double d, double e,
                     double cx, double cy,
                     double& u, double& v) {
    double norm = sqrt(x*x + y*y);
    if (norm < 1e-14) norm = 1e-14;
    
    // Our: theta = atan(-z / norm)
    double theta = atan(-z / norm);
    
    // Our: Horner's method, coefficients already reversed
    double rho = 0.0;
    for (size_t k = 0; k < world2cam_coeffs.size(); k++) {
        rho = rho * theta + world2cam_coeffs[k];
    }
    
    double uu = x / norm * rho;
    double vv = y / norm * rho;
    
    // Our affine: u = uu + vv*e + cx, v = uu*d + vv*c + cy
    u = uu + vv * e + cx;
    v = uu * d + vv * c + cy;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "Usage: ./ocam_verify <ocam_json_file>" << endl;
        return 1;
    }
    
    // Load Ocam parameters
    ifstream file(argv[1]);
    if (!file.is_open()) {
        cerr << "Cannot open: " << argv[1] << endl;
        return 1;
    }
    
    Json::Value root;
    Json::Reader reader;
    if (!reader.parse(file, root)) {
        cerr << "Failed to parse JSON" << endl;
        return 1;
    }
    
    const Json::Value& intrinsic = root["intrinsic_param"];
    
    // Load parameters
    double cx = intrinsic["principal_point"][0].asDouble();
    double cy = intrinsic["principal_point"][1].asDouble();
    double c = intrinsic["affine_c"].asDouble();
    double d = intrinsic["affine_d"].asDouble();
    double e = intrinsic["affine_e"].asDouble();
    
    vector<double> world2cam_poly;
    const Json::Value& world2cam = intrinsic["world2cam"];
    for (int i = 0; i < (int)world2cam.size(); i++)
        world2cam_poly.push_back(world2cam[i].asDouble());
    
    // Create reversed coefficients (our method)
    vector<double> world2cam_coeffs(world2cam_poly.size());
    for (size_t i = 0; i < world2cam_poly.size(); i++)
        world2cam_coeffs[i] = world2cam_poly[world2cam_poly.size() - 1 - i];
    
    cout << "========================================" << endl;
    cout << "Ocam Projection Verification" << endl;
    cout << "========================================" << endl;
    cout << "\nParameters:" << endl;
    cout << "  Principal point: (" << cx << ", " << cy << ")" << endl;
    cout << "  Affine: c=" << c << ", d=" << d << ", e=" << e << endl;
    cout << "  world2cam polynomial (" << world2cam_poly.size() << " coeffs):" << endl;
    cout << "    [";
    for (size_t i = 0; i < world2cam_poly.size(); i++) {
        if (i > 0) cout << ", ";
        cout << world2cam_poly[i];
    }
    cout << "]" << endl;
    
    // Test points in camera coordinates
    cout << "\n========================================" << endl;
    cout << "Test Projections (Camera Coordinates)" << endl;
    cout << "========================================" << endl;
    
    double test_points[][3] = {
        {0, 0, 1},      // Forward center
        {1, 0, 1},      // Right
        {-1, 0, 1},     // Left
        {0, 1, 1},      // Up
        {0, -1, 1},     // Down
        {0.5, 0.5, 1},  // Right-Up
        {2, 0, 1},      // Far right
        {0, 0, 2},      // Far forward
    };
    const char* point_names[] = {
        "Center", "Right", "Left", "Up", "Down", 
        "Right-Up", "Far Right", "Far Forward"
    };
    
    for (int i = 0; i < 8; i++) {
        double x = test_points[i][0];
        double y = test_points[i][1];
        double z = test_points[i][2];
        
        double u_cpac, v_cpac;
        double u_our, v_our;
        
        point2pixel_cpac(x, y, z, world2cam_poly, c, d, e, cx, cy, u_cpac, v_cpac);
        point2pixel_our(x, y, z, world2cam_coeffs, c, d, e, cx, cy, u_our, v_our);
        
        cout << "\n" << point_names[i] << " (" << x << ", " << y << ", " << z << "):" << endl;
        cout << "  CPAC:  (" << u_cpac << ", " << v_cpac << ")" << endl;
        cout << "  Ours:  (" << u_our << ", " << v_our << ")" << endl;
        
        double diff_u = abs(u_cpac - u_our);
        double diff_v = abs(v_cpac - v_our);
        
        if (diff_u > 0.01 || diff_v > 0.01) {
            cout << "  WARNING: Difference > 0.01! (" << diff_u << ", " << diff_v << ")" << endl;
        } else {
            cout << "  OK: Difference (" << diff_u << ", " << diff_v << ")" << endl;
        }
    }
    
    // Test theta/rho relationship
    cout << "\n========================================" << endl;
    cout << "Theta-Rho Relationship Test" << endl;
    cout << "========================================" << endl;
    
    for (double theta = -M_PI/3; theta <= M_PI/3; theta += 0.1) {
        // CPAC
        double rho_cpac = 0.0;
        for (int i = world2cam_poly.size() - 1; i >= 0; i--) {
            rho_cpac = rho_cpac * theta + world2cam_poly[i];
        }
        
        // Ours
        double rho_our = 0.0;
        for (size_t k = 0; k < world2cam_coeffs.size(); k++) {
            rho_our = rho_our * theta + world2cam_coeffs[k];
        }
        
        double diff = abs(rho_cpac - rho_our);
        if (diff > 0.01) {
            cout << "Theta=" << theta << ": rho_cpac=" << rho_cpac 
                 << ", rho_our=" << rho_our << " DIFF=" << diff << " ***" << endl;
        }
    }
    cout << "\nVerification complete!" << endl;
    
    return 0;
}
