#include <Simulator/SharedLibrary.h>

#include <dlfcn.h>

#include <stdexcept>
#include <string>
#include <utility>

namespace simulator
{

SharedLibrary::SharedLibrary(std::filesystem::path library_file)
    : library_file_(std::move(library_file))
{
    dlerror();
    handle_ = dlopen(library_file_.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle_ == nullptr) {
        const char* error = dlerror();
        throw std::runtime_error("Cannot load shared library '" +
                                 library_file_.string() + "': " +
                                 (error == nullptr ? "unknown dlopen error" : error));
    }
}

SharedLibrary::~SharedLibrary() noexcept
{
    close();
}

SharedLibrary::SharedLibrary(SharedLibrary&& other) noexcept
    : library_file_(std::move(other.library_file_)),
      handle_(std::exchange(other.handle_, nullptr))
{
}

SharedLibrary& SharedLibrary::operator=(SharedLibrary&& other) noexcept
{
    if (this != &other) {
        close();
        library_file_ = std::move(other.library_file_);
        handle_ = std::exchange(other.handle_, nullptr);
    }
    return *this;
}

const std::filesystem::path& SharedLibrary::path() const noexcept
{
    return library_file_;
}

bool SharedLibrary::isOpen() const noexcept
{
    return handle_ != nullptr;
}

void* SharedLibrary::nativeHandle() const noexcept
{
    return handle_;
}

void SharedLibrary::close() noexcept
{
    if (handle_ != nullptr) {
        dlclose(handle_);
        handle_ = nullptr;
    }
}

} // namespace simulator
