/**
 * @file SharedLibrary.h
 * @brief Declares RAII ownership of one dynamically loaded shared library.
 *
 * The wrapper pairs dlopen with dlclose and is deliberately move-only. Plugin
 * descriptors declare their factories after this handle so factory objects are
 * destroyed before the associated code is unloaded.
 */
#pragma once

#include <filesystem>

namespace simulator
{

/** @brief Move-only owner of a successfully loaded .so handle. */
class SharedLibrary final
{
public:
    /**
     * @brief Loads a shared library immediately.
     * @param library_file Readable .so file to load with local symbol visibility.
     * @throws std::runtime_error If dlopen fails.
     */
    explicit SharedLibrary(std::filesystem::path library_file);

    /** @brief Unloads the library when a handle is owned. */
    ~SharedLibrary() noexcept;

    /**
     * @brief Transfers ownership from another wrapper.
     * @param other Wrapper whose handle becomes empty.
     */
    SharedLibrary(SharedLibrary&& other) noexcept;

    /**
     * @brief Releases any current handle and transfers another one.
     * @param other Wrapper whose handle becomes empty.
     * @return This wrapper after the transfer.
     */
    SharedLibrary& operator=(SharedLibrary&& other) noexcept;

    /** @brief Shared-library handles cannot be copied. */
    SharedLibrary(const SharedLibrary&) = delete;

    /** @brief Shared-library handle ownership cannot be copy-assigned. */
    SharedLibrary& operator=(const SharedLibrary&) = delete;

    /**
     * @brief Returns the loaded library path.
     * @return Path passed to the constructor.
     */
    [[nodiscard]] const std::filesystem::path& path() const noexcept;

    /**
     * @brief Reports whether this wrapper currently owns a native handle.
     * @return true when the library is loaded.
     */
    [[nodiscard]] bool isOpen() const noexcept;

    /**
     * @brief Exposes the native handle for low-level diagnostics.
     * @return Opaque dlopen handle, or nullptr when empty.
     */
    [[nodiscard]] void* nativeHandle() const noexcept;

private:
    /** @brief Calls dlclose and clears the native handle without throwing. */
    void close() noexcept;

    std::filesystem::path library_file_; ///< Loaded .so path.
    void* handle_ = nullptr; ///< Opaque dlopen handle.
};

} // namespace simulator
