#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <grpcpp/grpcpp.h>
#include "cccad/geometry/geometry_service.hpp"

namespace {

std::string getenv_or(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || std::string(value).empty()) {
    return fallback;
  }
  return value;
}

} // namespace

int main() {
  const std::string host = getenv_or("GEOMETRY_SERVICE_HOST", "0.0.0.0");
  const std::string port = getenv_or("GEOMETRY_SERVICE_PORT", "50052");
  const std::string storage_root = getenv_or("GEOMETRY_STORAGE_ROOT", "/data/geometry");

  std::filesystem::create_directories(storage_root);

  cccad::geometry::ArtifactWriter writer(storage_root);
  cccad::geometry::GeometryKernelServiceImpl service(std::move(writer));

  const std::string address = host + ":" + port;
  grpc::ServerBuilder builder;
  builder.AddListeningPort(address, grpc::InsecureServerCredentials());
  builder.RegisterService(&service);

  std::unique_ptr<grpc::Server> server(builder.BuildAndStart());
  if (!server) {
    std::cerr << "failed to start cccad geometry service on " << address << std::endl;
    return 1;
  }

  std::cout << "cccad geometry service listening on " << address << std::endl;
  std::cout << "geometry storage root: " << storage_root << std::endl;

  server->Wait();
  return 0;
}
