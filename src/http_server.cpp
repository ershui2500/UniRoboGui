#include "g1_web/http_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/system/system_error.hpp>
#include <json/json.h>

#include "g1_web/json_serializer.hpp"
#include "g1_web/static_assets.hpp"

namespace g1_web {
namespace {

namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
using tcp = net::ip::tcp;

bool ReadFile(const std::filesystem::path& path, std::string& body) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return false;
  }
  std::ostringstream stream;
  stream << input.rdbuf();
  body = stream.str();
  return true;
}

std::string JsonResponse(const Json::Value& value) {
  Json::StreamWriterBuilder builder;
  builder["commentStyle"] = "None";
  builder["indentation"] = "";
  return Json::writeString(builder, value);
}

class WebSocketSession
    : public std::enable_shared_from_this<WebSocketSession> {
 public:
  WebSocketSession(tcp::socket socket, SnapshotStore& store,
                   std::chrono::milliseconds interval)
      : ws_(std::move(socket)),
        timer_(ws_.get_executor()),
        store_(store),
        interval_(interval) {}

  void Run(http::request<http::string_body> request) {
    ws_.set_option(
        websocket::stream_base::timeout::suggested(beast::role_type::server));
    ws_.set_option(websocket::stream_base::decorator(
        [](websocket::response_type& response) {
          response.set(http::field::server, "g1-web-control");
        }));
    websocket::permessage_deflate compression;
    compression.server_enable = true;
    compression.compLevel = 3;
    ws_.set_option(compression);
    ws_.read_message_max(1024);
    auto self = shared_from_this();
    ws_.async_accept(
        request, [self](beast::error_code error) { self->OnAccept(error); });
  }

 private:
  void OnAccept(beast::error_code error) {
    if (error) {
      return;
    }
    ws_.text(true);
    DoRead();
    WriteSnapshot();
  }

  void DoRead() {
    auto self = shared_from_this();
    ws_.async_read(read_buffer_,
                   [self](beast::error_code error, std::size_t) {
                     if (error) {
                       self->timer_.cancel();
                       return;
                     }
                     self->read_buffer_.consume(self->read_buffer_.size());
                     self->timer_.cancel();
                     self->ws_.async_close(
                         websocket::close_reason(
                             websocket::close_code::policy_error),
                         [self](beast::error_code) {});
                   });
  }

  void WriteSnapshot() {
    outgoing_ = SerializeSnapshot(store_.GetSnapshot());
    auto self = shared_from_this();
    ws_.async_write(net::buffer(outgoing_),
                    [self](beast::error_code error, std::size_t) {
                      if (error) {
                        self->timer_.cancel();
                        return;
                      }
                      self->ScheduleNext();
                    });
  }

  void ScheduleNext() {
    timer_.expires_after(interval_);
    auto self = shared_from_this();
    timer_.async_wait([self](beast::error_code error) {
      if (!error) {
        self->WriteSnapshot();
      }
    });
  }

  websocket::stream<tcp::socket> ws_;
  net::steady_timer timer_;
  beast::flat_buffer read_buffer_;
  SnapshotStore& store_;
  std::chrono::milliseconds interval_;
  std::string outgoing_;
};

class HttpSession : public std::enable_shared_from_this<HttpSession> {
 public:
  HttpSession(tcp::socket socket, SnapshotStore& store,
              VoiceService& voice_service,
              ControlService& control_service,
              PerceptionService& perception_service,
              CameraService& camera_service,
              std::filesystem::path web_root,
              std::chrono::milliseconds publish_interval)
      : socket_(std::move(socket)),
        store_(store),
        voice_service_(voice_service),
        control_service_(control_service),
        perception_service_(perception_service),
        camera_service_(camera_service),
        web_root_(std::move(web_root)),
        publish_interval_(publish_interval) {}

  void Run() {
    beast::error_code ignored;
    socket_.set_option(tcp::no_delay(true), ignored);
    ReadRequest();
  }

 private:
  void ReadRequest() {
    request_ = {};
    auto self = shared_from_this();
    http::async_read(
        socket_, buffer_, request_,
        [self](beast::error_code error, std::size_t) {
          if (!error) {
            self->HandleRequest();
          }
        });
  }

  void HandleRequest() {
    const std::string target(request_.target());
    if (websocket::is_upgrade(request_) && target == "/ws/telemetry") {
      std::make_shared<WebSocketSession>(
          std::move(socket_), store_, publish_interval_)
          ->Run(std::move(request_));
      return;
    }

    if (target == "/api/voice/tts" &&
        request_.method() == http::verb::post) {
      HandleTtsRequest();
      return;
    }
    if (target == "/api/voice/asr" &&
        request_.method() == http::verb::post) {
      HandleAsrRequest();
      return;
    }
    if (target == "/api/voice/volume" &&
        request_.method() == http::verb::post) {
      HandleVolumeRequest();
      return;
    }
    if (target == "/api/voice/llm/mode" &&
        request_.method() == http::verb::post) {
      HandleLlmModeRequest();
      return;
    }
    if (target == "/api/voice/llm/customer-config" &&
        request_.method() == http::verb::get) {
      HandleCustomerLlmConfigGet();
      return;
    }
    if (target == "/api/voice/llm/customer-config" &&
        request_.method() == http::verb::post) {
      HandleCustomerLlmConfigSet();
      return;
    }
    if (target == "/api/voice/llm/chat" &&
        request_.method() == http::verb::post) {
      HandleLlmChatRequest();
      return;
    }
    if (target == "/api/control/command" &&
        request_.method() == http::verb::post) {
      HandleControlRequest();
      return;
    }
    if (target == "/api/control/velocity" &&
        request_.method() == http::verb::post) {
      HandleVelocityRequest();
      return;
    }
    if (target == "/api/control/joint-debug/apply" &&
        request_.method() == http::verb::post) {
      HandleJointDebugApply();
      return;
    }
    if (target == "/api/control/joint-debug/stop" &&
        request_.method() == http::verb::post) {
      HandleJointDebugStop();
      return;
    }
    if (target == "/api/control/joint-debug/heartbeat" &&
        request_.method() == http::verb::post) {
      const auto result = control_service_.HeartbeatJointDebug();
      Json::Value response(Json::objectValue);
      response["accepted"] = result.accepted;
      response["error"] = result.error;
      Send(result.accepted ? http::status::ok : http::status::conflict,
           "application/json; charset=utf-8", JsonResponse(response));
      return;
    }
    if (target == "/api/control/joint-debug/teach/record" &&
        request_.method() == http::verb::post) {
      HandleJointTeachRequest("record");
      return;
    }
    if (target == "/api/control/joint-debug/teach/save" &&
        request_.method() == http::verb::post) {
      HandleJointTeachRequest("save");
      return;
    }
    if (target == "/api/control/joint-debug/teach/play" &&
        request_.method() == http::verb::post) {
      HandleJointTeachRequest("play");
      return;
    }
    if (target == "/api/control/joint-debug/teach/delete" &&
        request_.method() == http::verb::post) {
      HandleJointTeachRequest("delete");
      return;
    }
    if (target == "/api/control/joint-debug/teach/bind" &&
        request_.method() == http::verb::post) {
      HandleJointTeachRequest("bind");
      return;
    }
    if (target == "/api/perception/command" &&
        request_.method() == http::verb::post) {
      HandlePerceptionRequest();
      return;
    }
    if (target == "/api/perception/topic-config" &&
        request_.method() == http::verb::post) {
      HandleNavigationTopicRequest();
      return;
    }
    if (target == "/api/camera/command" &&
        request_.method() == http::verb::post) {
      HandleCameraRequest();
      return;
    }
    if (request_.method() != http::verb::get) {
      Send(http::status::method_not_allowed, "application/json; charset=utf-8",
           "{\"error\":\"method_not_allowed\"}");
      return;
    }

    const std::string map_download_prefix =
        "/api/perception/map-file?map_name=";
    if (target.rfind(map_download_prefix, 0) == 0) {
      HandleMapDownload(target.substr(map_download_prefix.size()));
      return;
    }

    if (target == "/api/health") {
      Send(http::status::ok, "application/json; charset=utf-8",
           SerializeHealth(store_.GetSnapshot()));
      return;
    }
    if (target == "/api/snapshot") {
      Send(http::status::ok, "application/json; charset=utf-8",
           SerializeSnapshot(store_.GetSnapshot()));
      return;
    }
    if (target == "/api/voice/status") {
      Send(http::status::ok, "application/json; charset=utf-8",
           SerializeVoiceStatus(store_.GetSnapshot()));
      return;
    }
    if (target == "/api/control/status") {
      Send(http::status::ok, "application/json; charset=utf-8",
           SerializeControlStatus(store_.GetSnapshot()));
      return;
    }
    if (target == "/api/control/joint-debug/status") {
      Send(http::status::ok, "application/json; charset=utf-8",
           control_service_.SerializeJointDebugStatus());
      return;
    }
    if (target == "/api/perception/status") {
      Send(http::status::ok, "application/json; charset=utf-8",
           perception_service_.SerializeStatus());
      return;
    }
    if (target == "/api/perception/frame") {
      Send(http::status::ok, "application/json; charset=utf-8",
           perception_service_.SerializeFrame());
      return;
    }
    if (target == "/api/perception/global-map") {
      Send(http::status::ok, "application/json; charset=utf-8",
           perception_service_.SerializeGlobalMap());
      return;
    }
    if (target == "/api/perception/navigation-scene") {
      Send(http::status::ok, "application/json; charset=utf-8",
           perception_service_.SerializeNavigationScene());
      return;
    }
    if (target == "/api/perception/topics") {
      Send(http::status::ok, "application/json; charset=utf-8",
           perception_service_.SerializeNavigationTopics());
      return;
    }
    if (target == "/api/camera/status") {
      Send(http::status::ok, "application/json; charset=utf-8",
           camera_service_.SerializeStatus());
      return;
    }
    if (target == "/api/camera/rgb/frame.jpg") {
      HandleCameraFrame("rgb");
      return;
    }
    if (target == "/api/camera/depth/frame.jpg") {
      HandleCameraFrame("depth");
      return;
    }

    std::filesystem::path static_path;
    if (!ResolveStaticAsset(web_root_, target, static_path)) {
      Send(http::status::not_found, "application/json; charset=utf-8",
           "{\"error\":\"not_found\"}");
      return;
    }

    std::string body;
    if (!ReadFile(static_path, body)) {
      Send(http::status::not_found, "application/json; charset=utf-8",
           "{\"error\":\"static_file_not_found\"}");
      return;
    }
    const bool immutable_asset =
        target.rfind("/assets/vendor/", 0) == 0 ||
        target.rfind("/assets/unitree/", 0) == 0;
    Send(http::status::ok, StaticContentType(static_path), std::move(body),
         immutable_asset ? "public, max-age=31536000, immutable" : "no-cache");
  }

  void HandleTtsRequest() {
    if (request_.body().size() > 4096) {
      Send(http::status::payload_too_large,
           "application/json; charset=utf-8",
           R"({"accepted":false,"error":"request_too_large"})");
      return;
    }

    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (!Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root.isMember("text") ||
        !root["text"].isString() ||
        (root.isMember("speaker_id") && !root["speaker_id"].isIntegral()) ||
        (root.isMember("tts_backend") && !root["tts_backend"].isString())) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }

    const std::string text = root["text"].asString();
    const std::int32_t speaker_id = root.get("speaker_id", -1).asInt();
    const std::string tts_backend = root.get("tts_backend", "").asString();
    const auto result =
        voice_service_.EnqueueTts(text, speaker_id, tts_backend);

    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["request_id"] = Json::UInt64(result.request_id);
    response["speaker_id"] = result.speaker_id;
    response["tts_backend"] = result.backend;
    response["error"] = result.error;
    if (result.accepted) {
      Send(http::status::accepted, "application/json; charset=utf-8",
           JsonResponse(response));
    } else if (result.error == "tts_queue_full") {
      Send(http::status::too_many_requests,
           "application/json; charset=utf-8", JsonResponse(response));
    } else if (result.error == "tts_busy" ||
               result.error == "audio_busy" ||
               result.error == "duplicate_tts_request") {
      Send(http::status::conflict,
           "application/json; charset=utf-8", JsonResponse(response));
    } else if (result.error == "voice_not_ready") {
      Send(http::status::service_unavailable,
           "application/json; charset=utf-8", JsonResponse(response));
    } else {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           JsonResponse(response));
    }
  }

  void HandleAsrRequest() {
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (request_.body().size() > 1024 ||
        !Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["enabled"].isBool()) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }
    const auto result = voice_service_.SetAsrEnabled(root["enabled"].asBool());
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["enabled"] = result.value == 1;
    response["api_result"] = result.api_result;
    response["error"] = result.error;
    Send(result.accepted ? http::status::ok
                         : result.error == "voice_not_ready"
                               ? http::status::service_unavailable
                               : http::status::bad_request,
         "application/json; charset=utf-8", JsonResponse(response));
  }

  void HandleVolumeRequest() {
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (request_.body().size() > 1024 ||
        !Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["volume"].isIntegral()) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }
    const auto result = voice_service_.SetVolume(root["volume"].asInt());
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["volume"] = result.value;
    response["api_result"] = result.api_result;
    response["error"] = result.error;
    Send(result.accepted ? http::status::ok
                         : result.error == "voice_not_ready"
                               ? http::status::service_unavailable
                               : http::status::bad_request,
         "application/json; charset=utf-8", JsonResponse(response));
  }

  static Json::Value CustomerVoiceConfigJson(
      const CustomerVoiceConfigResult& result) {
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["error"] = result.error;
    response["api_url"] = result.config.api_url;
    response["api_key"] = result.config.api_key;
    response["api_key_configured"] = result.config.api_key_configured;
    response["model"] = result.config.model;
    response["config_path"] = result.config.config_path;
    response["qa_replace_semantics"] = true;
    response["qa_delete_semantics"] = true;
    response["config_disk_sync"] = true;
    response["role_prompt"] = result.config.role_prompt;
    response["wake_word"] = result.config.wake_word;
    response["wake_enabled"] = result.config.wake_enabled;
    response["tts_backend"] = result.config.tts_backend;
    Json::Value entries(Json::arrayValue);
    for (const auto& entry : result.config.qa_entries) {
      Json::Value item(Json::objectValue);
      item["question"] = entry.question;
      item["answer"] = entry.answer;
      entries.append(item);
    }
    response["qa_entries"] = entries;
    return response;
  }

  void HandleCustomerLlmConfigGet() {
    const auto result = voice_service_.GetCustomerVoiceConfig();
    Send(result.accepted ? http::status::ok : http::status::service_unavailable,
         "application/json; charset=utf-8",
         JsonResponse(CustomerVoiceConfigJson(result)));
  }

  void HandleCustomerLlmConfigSet() {
    if (request_.body().size() > 512 * 1024) {
      Send(http::status::payload_too_large,
           "application/json; charset=utf-8",
           R"({"accepted":false,"error":"request_too_large"})");
      return;
    }
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (!Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["role_prompt"].isString() ||
        !root["wake_word"].isString() || !root["wake_enabled"].isBool() ||
        !root["tts_backend"].isString() || !root["qa_entries"].isArray() ||
        (root.isMember("api_url") && !root["api_url"].isString()) ||
        (root.isMember("api_key") && !root["api_key"].isString()) ||
        (root.isMember("model") && !root["model"].isString()) ||
        (root.isMember("preserve_api_key") &&
         !root["preserve_api_key"].isBool())) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }

    const auto current = voice_service_.GetCustomerVoiceConfig();
    CustomerVoiceConfig config = current.config;
    config.update_api_config = root.isMember("api_url") ||
                               root.isMember("api_key") ||
                               root.isMember("model") ||
                               root.isMember("preserve_api_key");
    config.api_url = root.get("api_url", config.api_url).asString();
    config.model = root.get("model", config.model).asString();
    if (root.isMember("api_key")) {
      config.api_key = root["api_key"].asString();
      config.preserve_api_key = root.get("preserve_api_key", false).asBool();
    } else {
      config.api_key.clear();
      config.preserve_api_key = true;
    }
    config.role_prompt = root["role_prompt"].asString();
    config.wake_word = root["wake_word"].asString();
    config.wake_enabled = root["wake_enabled"].asBool();
    config.tts_backend = root["tts_backend"].asString();
    config.qa_entries.clear();
    const Json::Value& entries = root["qa_entries"];
    for (Json::ArrayIndex i = 0; i < entries.size(); ++i) {
      if (!entries[i].isObject() || !entries[i]["question"].isString() ||
          !entries[i]["answer"].isString()) {
        Send(http::status::bad_request, "application/json; charset=utf-8",
             R"({"accepted":false,"error":"invalid_json"})");
        return;
      }
      config.qa_entries.push_back(
          {entries[i]["question"].asString(), entries[i]["answer"].asString()});
    }

    const auto result = voice_service_.SetCustomerVoiceConfig(config);
    http::status status = http::status::bad_request;
    if (result.accepted) {
      status = http::status::ok;
    } else if (result.error == "voice_not_ready") {
      status = http::status::service_unavailable;
    }
    Send(status, "application/json; charset=utf-8",
         JsonResponse(CustomerVoiceConfigJson(result)));
  }

  void HandleLlmModeRequest() {
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (request_.body().size() > 16384 ||
        !Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["mode"].isString() ||
        (root.isMember("api_url") && !root["api_url"].isString()) ||
        (root.isMember("api_key") && !root["api_key"].isString()) ||
        (root.isMember("model") && !root["model"].isString()) ||
        (root.isMember("preserve_api_key") &&
         !root["preserve_api_key"].isBool())) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }

    const auto result = voice_service_.SetLlmMode(
        root["mode"].asString(), root.get("api_url", "").asString(),
        root.get("api_key", "").asString(),
        root.get("model", "").asString(),
        root.get("preserve_api_key", false).asBool());
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["mode"] = result.mode;
    response["api_result"] = result.api_result;
    response["error"] = result.error;

    http::status status = http::status::bad_request;
    if (result.accepted) {
      status = http::status::ok;
    } else if (result.error == "voice_not_ready") {
      status = http::status::service_unavailable;
    } else if (result.error == "customer_api_unavailable") {
      status = http::status::not_implemented;
    } else if (result.error == "chat_go_enable_failed" ||
               result.error == "chat_go_disable_failed" ||
               result.error == "customer_api_key_endpoint_changed" ||
               result.error == "customer_api_not_configured") {
      status = http::status::conflict;
    }
    Send(status, "application/json; charset=utf-8", JsonResponse(response));
  }

  void HandleLlmChatRequest() {
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (request_.body().size() > 8192 ||
        !Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["mode"].isString() ||
        !root["message"].isString() ||
        (root.isMember("auto_tts") && !root["auto_tts"].isBool())) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }

    const auto result = voice_service_.ChatWithLlm(
        root["mode"].asString(), root["message"].asString(),
        root.get("auto_tts", false).asBool());
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["pending"] = result.pending;
    response["request_id"] = Json::Int64(result.request_id);
    response["mode"] = result.mode;
    response["http_status"] = Json::Int64(result.http_status);
    response["tts_request_id"] = Json::UInt64(result.tts_request_id);
    response["response"] = result.response;
    response["response_source"] = result.response_source;
    response["error"] = result.error;
    response["tts_error"] = result.tts_error;

    http::status status = http::status::bad_request;
    if (result.accepted) {
      status = http::status::ok;
    } else if (result.error == "voice_not_ready") {
      status = http::status::service_unavailable;
    } else if (result.error == "llm_mode_mismatch" ||
               result.error == "customer_llm_not_active" ||
               result.error == "customer_api_not_configured" ||
               result.error == "builtin_llm_not_active" ||
               result.error == "builtin_llm_busy") {
      status = http::status::conflict;
    } else if (result.error == "builtin_gpt_channel_unavailable") {
      status = http::status::service_unavailable;
    } else if (result.error == "customer_api_unavailable") {
      status = http::status::not_implemented;
    } else if (result.error.rfind("customer_api_", 0) == 0 ||
               result.http_status != 0) {
      status = http::status::bad_gateway;
    }
    Send(status, "application/json; charset=utf-8", JsonResponse(response));
  }

  void HandleControlRequest() {
    if (request_.body().size() > 4096) {
      Send(http::status::payload_too_large,
           "application/json; charset=utf-8",
           R"({"accepted":false,"error":"request_too_large"})");
      return;
    }

    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (!Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["request_key"].isString() ||
        !root["category"].isString() || !root["command"].isString() ||
        !root["confirmed"].isBool() ||
        (root.isMember("argument") && !root["argument"].isIntegral()) ||
        (root.isMember("action_name") && !root["action_name"].isString())) {
      Send(http::status::bad_request,
           "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }

    const auto result = control_service_.Submit(
        root["request_key"].asString(), root["category"].asString(),
        root["command"].asString(), root.get("argument", 0).asInt(),
        root["confirmed"].asBool(),
        root.get("action_name", "").asString());

    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["duplicate"] = result.duplicate;
    response["request_id"] = Json::UInt64(result.request_id);
    response["error"] = result.error;
    if (result.accepted) {
      Send(http::status::accepted,
           "application/json; charset=utf-8", JsonResponse(response));
    } else if (result.error == "control_not_ready") {
      Send(http::status::service_unavailable,
           "application/json; charset=utf-8", JsonResponse(response));
    } else if (result.error == "control_busy" ||
               result.error == "sport_state_stale" ||
               result.error == "robot_not_static" ||
               result.error == "motion_active" ||
               result.error == "arm_action_fsm_not_allowed" ||
               result.error == "arm_action_robot_not_static" ||
               result.error == "arm_action_not_available_on_firmware" ||
               result.error == "teach_action_not_available_on_firmware" ||
               result.error == "arm_action_not_supported_by_model") {
      Send(http::status::conflict,
           "application/json; charset=utf-8", JsonResponse(response));
    } else {
      Send(http::status::bad_request,
           "application/json; charset=utf-8", JsonResponse(response));
    }
  }

  void HandleVelocityRequest() {
    if (request_.body().size() > 2048) {
      Send(http::status::payload_too_large,
           "application/json; charset=utf-8",
           R"({"accepted":false,"error":"request_too_large"})");
      return;
    }

    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (!Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["active"].isBool() ||
        !root["vx"].isNumeric() || !root["vy"].isNumeric() ||
        !root["vyaw"].isNumeric() ||
        (root["active"].asBool() &&
         (!root.isMember("speed_mode") ||
          !root["speed_mode"].isIntegral()))) {
      Send(http::status::bad_request,
           "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }

    const auto result = control_service_.SubmitVelocity(
        root["vx"].asFloat(), root["vy"].asFloat(),
        root["vyaw"].asFloat(), root.get("speed_mode", 0).asInt(),
        root["active"].asBool());
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["sequence"] = Json::UInt64(result.sequence);
    response["error"] = result.error;
    if (result.accepted) {
      Send(http::status::accepted,
           "application/json; charset=utf-8", JsonResponse(response));
    } else if (result.error == "control_not_ready") {
      Send(http::status::service_unavailable,
           "application/json; charset=utf-8", JsonResponse(response));
    } else if (result.error == "control_busy" ||
               result.error == "sport_state_stale" ||
               result.error == "motion_fsm_not_allowed" ||
               result.error == "speed_mode_requires_walkrun") {
      Send(http::status::conflict,
           "application/json; charset=utf-8", JsonResponse(response));
    } else {
      Send(http::status::bad_request,
           "application/json; charset=utf-8", JsonResponse(response));
    }
  }

  void HandleJointDebugApply() {
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (request_.body().size() > 32768 ||
        !Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["mode"].isString() ||
        !root["confirmed"].isBool() || !root["joints"].isArray()) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }
    std::vector<std::pair<std::size_t, float>> targets;
    for (const auto& joint : root["joints"]) {
      if (!joint.isObject() || !joint["index"].isUInt() ||
          !joint["q"].isNumeric()) {
        Send(http::status::bad_request, "application/json; charset=utf-8",
             R"({"accepted":false,"error":"invalid_json"})");
        return;
      }
      targets.emplace_back(joint["index"].asUInt(), joint["q"].asFloat());
    }
    const auto result = control_service_.ApplyJointDebug(
        root["mode"].asString(), targets, root["confirmed"].asBool());
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["error"] = result.error;
    const bool conflict = result.error == "control_busy" ||
                          result.error == "debug_mode_required" ||
                          result.error == "upper_body_fsm_not_allowed" ||
                          result.error == "sport_state_stale" ||
                          result.error == "lowstate_unavailable" ||
                          result.error == "pr_mode_required";
    Send(result.accepted ? http::status::accepted
                         : conflict ? http::status::conflict
                                    : http::status::bad_request,
         "application/json; charset=utf-8", JsonResponse(response));
  }

  void HandleJointDebugStop() {
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (request_.body().size() > 1024 ||
        !Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["confirmed"].isBool()) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }
    const auto result =
        control_service_.StopJointDebug(root["confirmed"].asBool());
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["error"] = result.error;
    Send(result.accepted ? http::status::ok : http::status::bad_request,
         "application/json; charset=utf-8", JsonResponse(response));
  }

  void HandleJointTeachRequest(const std::string& operation) {
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    const bool binding_request = operation == "bind";
    if (request_.body().size() > 1024 ||
        !Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() ||
        (!binding_request && !root["confirmed"].isBool()) ||
        (operation != "save" && !root["name"].isString()) ||
        (binding_request && !root["binding"].isString()) ||
        (operation == "save" && root.isMember("release_control") &&
         !root["release_control"].isBool())) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }
    JointDebugSubmitResult result;
    if (operation == "record") {
      result = control_service_.StartJointTeachRecording(
          root["name"].asString(), root["confirmed"].asBool());
    } else if (operation == "save") {
      result = control_service_.FinishJointTeachRecording(
          root["confirmed"].asBool(),
          root.get("release_control", true).asBool());
    } else if (operation == "play") {
      result = control_service_.PlayJointTeachAction(
          root["name"].asString(), root["confirmed"].asBool());
    } else if (operation == "delete") {
      result = control_service_.DeleteJointTeachAction(
          root["name"].asString(), root["confirmed"].asBool());
    } else {
      result = control_service_.SetJointTeachRemoteBinding(
          root["name"].asString(), root["binding"].asString());
    }
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["error"] = result.error;
    const bool conflict = result.error == "control_busy" ||
                          result.error == "upper_body_fsm_not_allowed" ||
                          result.error == "sport_state_stale" ||
                          result.error == "lowstate_unavailable" ||
                          result.error == "joint_teach_model_mismatch" ||
                          result.error == "joint_teach_remote_binding_in_use";
    Send(result.accepted ? http::status::accepted
                         : conflict ? http::status::conflict
                                    : http::status::bad_request,
         "application/json; charset=utf-8", JsonResponse(response));
  }

  void HandlePerceptionRequest() {
    if (request_.body().size() > 4096) {
      Send(http::status::payload_too_large,
           "application/json; charset=utf-8",
           R"({"accepted":false,"error":"request_too_large"})");
      return;
    }

    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (!Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["request_key"].isString() ||
        !root["command"].isString() || !root["confirmed"].isBool() ||
        (root.isMember("map_name") && !root["map_name"].isString())) {
      Send(http::status::bad_request,
           "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }

    PerceptionRequest perception_request;
    perception_request.request_key = root["request_key"].asString();
    perception_request.command = root["command"].asString();
    perception_request.map_name = root.get("map_name", "").asString();
    perception_request.confirmed = root["confirmed"].asBool();
    if (perception_request.command == "navigate" ||
        perception_request.command == "load_map" ||
        perception_request.command == "initialize_pose") {
      if (!root["pose"].isObject() || !root["pose"]["x"].isNumeric() ||
          !root["pose"]["y"].isNumeric() ||
          !root["pose"]["z"].isNumeric() ||
          !root["pose"]["yaw"].isNumeric()) {
        Send(http::status::bad_request,
             "application/json; charset=utf-8",
             R"({"accepted":false,"error":"invalid_pose_json"})");
        return;
      }
      const double yaw = root["pose"]["yaw"].asDouble();
      perception_request.pose.x = root["pose"]["x"].asDouble();
      perception_request.pose.y = root["pose"]["y"].asDouble();
      perception_request.pose.z = root["pose"]["z"].asDouble();
      perception_request.pose.qz = std::sin(yaw * 0.5);
      perception_request.pose.qw = std::cos(yaw * 0.5);
    }

    const auto result = perception_service_.Submit(perception_request);
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["duplicate"] = result.duplicate;
    response["lidar_started"] = result.lidar_started;
    response["slam_started"] = result.slam_started;
    response["request_id"] = Json::UInt64(result.request_id);
    response["api_result"] = result.api_result;
    response["error"] = result.error;
    response["api_response"] = result.response;
    const std::string body = JsonResponse(response);
    if (result.accepted) {
      Send(http::status::accepted, "application/json; charset=utf-8", body);
    } else if (result.error == "perception_not_ready") {
      Send(http::status::service_unavailable,
           "application/json; charset=utf-8", body);
    } else if (result.error == "navigation_runtime_disabled") {
      Send(http::status::forbidden, "application/json; charset=utf-8", body);
    } else if (result.error == "slam_pose_stale" ||
               result.error == "goal_over_10m" ||
               result.error == "slam_api_failed" ||
               result.error == "invalid_slam_state") {
      Send(http::status::conflict, "application/json; charset=utf-8", body);
    } else {
      Send(http::status::bad_request, "application/json; charset=utf-8", body);
    }
  }

  void HandleMapDownload(const std::string& map_name) {
    const std::string path = ResolvePerceptionMapPath(map_name);
    const std::string download_path = ResolvePerceptionMapDownloadPath(map_name);
    if (path.empty() || download_path.empty()) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"error":"invalid_map_name"})");
      return;
    }
    std::string body;
    if (!perception_service_.GetTopicGlobalMapPcd(map_name, body) &&
        !ReadFile(download_path, body) && !ReadFile(path, body)) {
      Send(http::status::not_found, "application/json; charset=utf-8",
           R"({"error":"map_file_not_found"})");
      return;
    }
    Send(http::status::ok, "application/octet-stream", std::move(body),
         "no-store", "attachment; filename=\"" + map_name + "\"");
  }

  void HandleCameraFrame(const std::string& stream) {
    const auto frame = camera_service_.GetFrame(stream);
    if (!frame.available) {
      Send(http::status::service_unavailable,
           "application/json; charset=utf-8",
           R"({"error":"camera_frame_unavailable"})");
      return;
    }
    Send(http::status::ok, "image/jpeg",
         std::string(reinterpret_cast<const char*>(frame.jpeg.data()),
                     frame.jpeg.size()));
  }

  void HandleNavigationTopicRequest() {
    if (request_.body().size() > 4096) {
      Send(http::status::payload_too_large,
           "application/json; charset=utf-8",
           R"({"accepted":false,"error":"request_too_large"})");
      return;
    }
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (!Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["kind"].isString() ||
        !root["topic"].isString()) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }
    std::string error;
    const bool accepted = perception_service_.ConfigureNavigationTopic(
        root["kind"].asString(), root["topic"].asString(), error);
    Json::Value response(Json::objectValue);
    response["accepted"] = accepted;
    response["error"] = error;
    Send(accepted ? http::status::ok : http::status::bad_request,
         "application/json; charset=utf-8", JsonResponse(response));
  }

  void HandleCameraRequest() {
    if (request_.body().size() > 4096) {
      Send(http::status::payload_too_large,
           "application/json; charset=utf-8",
           R"({"accepted":false,"error":"request_too_large"})");
      return;
    }
    Json::CharReaderBuilder builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream stream(request_.body());
    if (!Json::parseFromStream(builder, stream, &root, &parse_errors) ||
        !root.isObject() || !root["request_key"].isString() ||
        !root["command"].isString() || !root["confirmed"].isBool() ||
        (root.isMember("rgb_source") && !root["rgb_source"].isString()) ||
        (root.isMember("depth_source") &&
         !root["depth_source"].isString())) {
      Send(http::status::bad_request, "application/json; charset=utf-8",
           R"({"accepted":false,"error":"invalid_json"})");
      return;
    }
    CameraRequest camera_request;
    camera_request.request_key = root["request_key"].asString();
    camera_request.command = root["command"].asString();
    camera_request.confirmed = root["confirmed"].asBool();
    camera_request.rgb_source = root.get("rgb_source", "").asString();
    camera_request.depth_source = root.get("depth_source", "").asString();
    const auto result = camera_service_.Submit(camera_request);
    Json::Value response(Json::objectValue);
    response["accepted"] = result.accepted;
    response["duplicate"] = result.duplicate;
    response["request_id"] = Json::UInt64(result.request_id);
    response["error"] = result.error;
    const std::string body = JsonResponse(response);
    if (result.accepted) {
      Send(http::status::accepted, "application/json; charset=utf-8", body);
    } else if (result.error == "opencv_not_available" ||
               result.error == "librealsense2_not_available") {
      Send(http::status::service_unavailable,
           "application/json; charset=utf-8", body);
    } else {
      Send(http::status::bad_request, "application/json; charset=utf-8", body);
    }
  }

  void Send(http::status status, const std::string& content_type,
            std::string body,
            const std::string& cache_control = "no-store",
            const std::string& content_disposition = "") {
    auto response =
        std::make_shared<http::response<http::string_body>>(status,
                                                            request_.version());
    response->set(http::field::server, "g1-web-control");
    response->set(http::field::content_type, content_type);
    response->set(http::field::cache_control, cache_control);
    if (!content_disposition.empty()) {
      response->set(http::field::content_disposition, content_disposition);
    }
    response->set("X-Content-Type-Options", "nosniff");
    response->set("X-Frame-Options", "DENY");
    response->set("Referrer-Policy", "no-referrer");
    response->set("Content-Security-Policy",
                  "default-src 'self'; connect-src 'self' ws: wss:; "
                  "style-src 'self'; script-src 'self'; "
                  "img-src 'self' data: blob:");
    response->body() = std::move(body);
    response->keep_alive(request_.keep_alive());
    response->prepare_payload();

    auto self = shared_from_this();
    http::async_write(
        socket_, *response,
        [self, response](beast::error_code error, std::size_t) {
          if (!error && response->keep_alive()) {
            self->ReadRequest();
            return;
          }
          beast::error_code ignored;
          self->socket_.shutdown(tcp::socket::shutdown_send, ignored);
        });
  }

  tcp::socket socket_;
  beast::flat_buffer buffer_;
  http::request<http::string_body> request_;
  SnapshotStore& store_;
  VoiceService& voice_service_;
  ControlService& control_service_;
  PerceptionService& perception_service_;
  CameraService& camera_service_;
  std::filesystem::path web_root_;
  std::chrono::milliseconds publish_interval_;
};

class Listener : public std::enable_shared_from_this<Listener> {
 public:
  Listener(net::io_context& io_context, const tcp::endpoint& endpoint,
           SnapshotStore& store, VoiceService& voice_service,
           ControlService& control_service,
           PerceptionService& perception_service,
           CameraService& camera_service,
           std::filesystem::path web_root,
           std::chrono::milliseconds publish_interval)
      : io_context_(io_context),
        acceptor_(net::make_strand(io_context)),
        store_(store),
        voice_service_(voice_service),
        control_service_(control_service),
        perception_service_(perception_service),
        camera_service_(camera_service),
        web_root_(std::move(web_root)),
        publish_interval_(publish_interval) {
    beast::error_code error;
    acceptor_.open(endpoint.protocol(), error);
    if (error) {
      throw boost::system::system_error(error);
    }
    acceptor_.set_option(net::socket_base::reuse_address(true), error);
    if (error) {
      throw boost::system::system_error(error);
    }
    acceptor_.bind(endpoint, error);
    if (error) {
      throw boost::system::system_error(error);
    }
    acceptor_.listen(net::socket_base::max_listen_connections, error);
    if (error) {
      throw boost::system::system_error(error);
    }
  }

  void Run() { Accept(); }

 private:
  void Accept() {
    auto self = shared_from_this();
    acceptor_.async_accept(
        net::make_strand(io_context_),
        [self](beast::error_code error, tcp::socket socket) {
          if (!error) {
            std::make_shared<HttpSession>(
                std::move(socket), self->store_, self->voice_service_,
                self->control_service_,
                self->perception_service_, self->camera_service_,
                self->web_root_,
                self->publish_interval_)
                ->Run();
          }
          if (self->acceptor_.is_open()) {
            self->Accept();
          }
        });
  }

  net::io_context& io_context_;
  tcp::acceptor acceptor_;
  SnapshotStore& store_;
  VoiceService& voice_service_;
  ControlService& control_service_;
  PerceptionService& perception_service_;
  CameraService& camera_service_;
  std::filesystem::path web_root_;
  std::chrono::milliseconds publish_interval_;
};

}  // namespace

class HttpServer::Impl {
 public:
  Impl(SnapshotStore& store, VoiceService& voice_service,
       ControlService& control_service,
       PerceptionService& perception_service,
       CameraService& camera_service,
       std::string bind_address, std::uint16_t port,
       std::string web_root, unsigned int publish_hz)
      : store_(store),
        voice_service_(voice_service),
        control_service_(control_service),
        perception_service_(perception_service),
        camera_service_(camera_service),
        bind_address_(std::move(bind_address)),
        port_(port),
        web_root_(std::move(web_root)),
        publish_hz_(publish_hz) {}

  bool Start(std::string& error) {
    if (!threads_.empty()) {
      error = "HTTP server is already running";
      return false;
    }
    try {
      if (!std::filesystem::is_directory(web_root_)) {
        error = "Web root is not a directory: " + web_root_;
        return false;
      }
      io_context_.restart();
      const auto address = net::ip::make_address(bind_address_);
      const auto interval = std::chrono::milliseconds(
          std::max(1U, 1000U / std::max(1U, publish_hz_)));
      listener_ = std::make_shared<Listener>(
          io_context_, tcp::endpoint{address, port_}, store_, voice_service_,
          control_service_, perception_service_, camera_service_,
          web_root_, interval);
      listener_->Run();
      constexpr std::size_t kHttpWorkerCount = 4;
      threads_.reserve(kHttpWorkerCount);
      for (std::size_t index = 0; index < kHttpWorkerCount; ++index) {
        threads_.emplace_back([this] { io_context_.run(); });
      }
      error.clear();
      return true;
    } catch (const std::exception& exception) {
      error = exception.what();
      io_context_.stop();
      for (auto& thread : threads_) {
        if (thread.joinable()) thread.join();
      }
      threads_.clear();
      listener_.reset();
      return false;
    }
  }

  void Stop() {
    io_context_.stop();
    for (auto& thread : threads_) {
      if (thread.joinable()) thread.join();
    }
    threads_.clear();
    listener_.reset();
  }

 private:
  SnapshotStore& store_;
  VoiceService& voice_service_;
  ControlService& control_service_;
  PerceptionService& perception_service_;
  CameraService& camera_service_;
  std::string bind_address_;
  std::uint16_t port_;
  std::string web_root_;
  unsigned int publish_hz_;
  net::io_context io_context_{4};
  std::shared_ptr<Listener> listener_;
  std::vector<std::thread> threads_;
};

HttpServer::HttpServer(SnapshotStore& store, VoiceService& voice_service,
                       ControlService& control_service,
                       PerceptionService& perception_service,
                       CameraService& camera_service,
                       std::string bind_address,
                       std::uint16_t port, std::string web_root,
                       unsigned int publish_hz)
    : impl_(std::make_unique<Impl>(
          store, voice_service, control_service, perception_service,
          camera_service, std::move(bind_address), port,
          std::move(web_root),
          publish_hz)) {}

HttpServer::~HttpServer() { Stop(); }

bool HttpServer::Start(std::string& error) { return impl_->Start(error); }

void HttpServer::Stop() { impl_->Stop(); }

}  // namespace g1_web
