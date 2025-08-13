#include "Geode/Enums.hpp"
#include "Geode/binding/EditorUI.hpp"
#include "Geode/binding/EndLevelLayer.hpp"
#include "Geode/binding/FMODAudioEngine.hpp"
#include "Geode/binding/GJBaseGameLayer.hpp"
#include "Geode/binding/GameManager.hpp"
#include "Geode/binding/GameObject.hpp"
#include "Geode/binding/GameToolbox.hpp"
#include "Geode/binding/LevelEditorLayer.hpp"
#include "Geode/binding/MenuLayer.hpp"
#include "Geode/binding/PlayLayer.hpp"
#include "Geode/binding/PlayerObject.hpp"
#include "Geode/cocos/CCDirector.h"
#include "Geode/cocos/actions/CCActionInstant.h"
#include "Geode/cocos/actions/CCActionInterval.h"
#include "Geode/cocos/cocoa/CCGeometry.h"
#include "Geode/cocos/cocoa/CCObject.h"
#include "Geode/cocos/draw_nodes/CCDrawNode.h"
#include "Geode/cocos/particle_nodes/CCParticleSystemQuad.h"
#include "Geode/loader/Log.hpp"
#include "ccTypes.h"
#include "fmod.hpp"
#include "fmod_common.h"
#include <Geode/Geode.hpp>
#include <Geode/modify/PlayLayer.hpp>
#include <Geode/modify/PlayerObject.hpp>
#include <Geode/modify/EndLevelLayer.hpp>
#include <Geode/modify/EditorUI.hpp>
#include <Geode/modify/PlayerObject.hpp>
#include <Geode/modify/GJBaseGameLayer.hpp>
#include <Geode/modify/LevelEditorLayer.hpp>
#include <Geode/modify/MenuLayer.hpp>
#include <cmath>
#include <csignal>
#include <optional>

using namespace geode::prelude;

namespace FDGlobal {
    std::vector<std::pair<GameObject*, CCPoint>> hiddenGameObjects;
    unsigned int oldObjectAmount = 0;
    unsigned int newObjectAmount = 0;

    CCDrawNode *drawNode = nullptr;

    FMOD::ChannelGroup *grp = nullptr;

    GameObject *last_collided_obj = nullptr;

    bool objectExists(GameObject *obj, const std::vector<GameObject*> &objectArray) {
        for (GameObject *o : objectArray) {
            if (o == obj) return true;
        }
        return false;
    }
    bool objectExists(GameObject *obj) {
        for (auto &pair : FDGlobal::hiddenGameObjects) {
            if (pair.first == obj) return true;
        }
        return false;
    }

    struct CCPointD {
        double x = 0.0;
        double y = 0.0;
    };

    struct Polygon {
        std::vector<CCPointD> points;
    };

    CCPointD getPolygonMiddle(const Polygon &polygon) {
        double cx = 0.0, cy = 0.0;
        double area = 0.0;
        int n = polygon.points.size();

        for (int i = 0; i < n; i++) {
            int j = (i + 1) % n;
            double cross = polygon.points[i].x * polygon.points[j].y - polygon.points[j].x * polygon.points[i].y;
            cx += (polygon.points[i].x + polygon.points[j].x) * cross;
            cy += (polygon.points[i].y + polygon.points[j].y) * cross;
            area += cross;
        }

        area /= 2.0;
        cx /= (6.0 * area);
        cy /= (6.0 * area);

        return {cx, cy};
    }

    CCPointD rotatePoint(const CCPointD &p, double angle) {
        double cosA = cos(angle);
        double sinA = sin(angle);
        return {
            p.x * cosA - p.y * sinA,
            p.x * sinA + p.y * cosA
        };
    }

    Polygon rotatePolAroundPol(const Polygon &polygonA, const Polygon &polygonB, double angle) {
        if (polygonA.points.empty() || polygonB.points.empty()) return polygonA;

        CCPointD origin = getPolygonMiddle(polygonB);

        std::vector<CCPointD> rotated;

        for (const CCPointD& p : polygonA.points) {
            CCPointD translated = {p.x - origin.x, p.y - origin.y};
            CCPointD rotatedPoint = rotatePoint(translated, angle);

            rotated.push_back({
                rotatedPoint.x + origin.x,
                rotatedPoint.y + origin.y
            });
        }

        return {rotated};
    }

    Polygon rotatePolygon(const Polygon &polygon, double angle) {
        return rotatePolAroundPol(polygon, polygon, angle);
    }

    bool IsPolygonsIntersecting(const Polygon &a, const Polygon &b) {
        // Create an array of both polygons to check edges from both
        const std::array<const Polygon*, 2> polygons = {&a, &b};

        for (const auto polygon : polygons) {
            for (size_t i1 = 0; i1 < polygon->points.size(); i1++) {
                size_t i2 = (i1 + 1) % polygon->points.size();
                const auto& p1 = polygon->points[i1];
                const auto& p2 = polygon->points[i2];

                // Calculate normal vector for the edge
                CCPointD normal = {p2.y - p1.y, p1.x - p2.x};

                // Project polygon A points
                std::optional<double> minA, maxA;
                for (const auto& p : a.points) {
                    double projected = normal.x * p.x + normal.y * p.y;
                    if (!minA || projected < *minA) minA = projected;
                    if (!maxA || projected > *maxA) maxA = projected;
                }

                // Project polygon B points
                std::optional<double> minB, maxB;
                for (const auto& p : b.points) {
                    double projected = normal.x * p.x + normal.y * p.y;
                    if (!minB || projected < *minB) minB = projected;
                    if (!maxB || projected > *maxB) maxB = projected;
                }

                // Check for separation along this axis
                if (*maxA < *minB || *maxB < *minA) {
                    return false;
                }
            }
        }
        return true;
    }

    std::optional<Polygon> getCustomPolygon(GameObject *obj) {
        if (obj->m_objectID == 8 || obj->m_objectID == 144 || obj->m_objectID == 177 || obj->m_objectID == 216) {
            std::vector<CCPointD> triangle = {
                {
                    (double)(obj->m_positionX) - (15.0 * obj->getScaleX()), // lower left
                    (double)(obj->m_positionY) - (15.0 * obj->getScaleY())
                },
                {
                    (double)(obj->m_positionX) + (15.0 * obj->getScaleX()), // lower right
                    (double)(obj->m_positionY) - (15.0 * obj->getScaleY())
                },
                {
                    (double)(obj->m_positionX), // upper center
                    (double)(obj->m_positionY) + (15.0 * obj->getScaleY())
                },
            };

            std::vector<CCPointD> trianglePolygon = {triangle[0], triangle[2], triangle[1]};
            auto box = obj->getOrientedBox();
            std::vector<CCPointD> hitboxPolygon = {
                {(double)(box->m_corners[0].x), (double)(box->m_corners[0].y)},
                {(double)(box->m_corners[1].x), (double)(box->m_corners[1].y)},
                {(double)(box->m_corners[2].x), (double)(box->m_corners[2].y)},
                {(double)(box->m_corners[3].x), (double)(box->m_corners[3].y)}
            };

            return FDGlobal::rotatePolAroundPol({trianglePolygon}, {hitboxPolygon}, (360.0 - obj->getRotation()) * (M_PI / 180.0));
        } else if (obj->m_objectID == 103 || obj->m_objectID == 145 || obj->m_objectID == 179 || obj->m_objectID == 218) {
            std::vector<CCPointD> triangle = {
                {
                    (double)(obj->m_positionX) - (15.0 * obj->getScaleX() * 0.6), // lower left
                    (double)(obj->m_positionY) - (15.0 * obj->getScaleY() * 0.6)
                },
                {
                    (double)(obj->m_positionX) + (15.0 * obj->getScaleX() * 0.6), // lower right
                    (double)(obj->m_positionY) - (15.0 * obj->getScaleY() * 0.6)
                },
                {
                    (double)(obj->m_positionX), // upper center
                    (double)(obj->m_positionY) + (15.0 * obj->getScaleY() * 0.6)
                },
            };

            std::vector<CCPointD> trianglePolygon = {triangle[0], triangle[2], triangle[1]};
            auto box = obj->getOrientedBox();
            std::vector<CCPointD> hitboxPolygon = {
                {(double)(box->m_corners[0].x), (double)(box->m_corners[0].y)},
                {(double)(box->m_corners[1].x), (double)(box->m_corners[1].y)},
                {(double)(box->m_corners[2].x), (double)(box->m_corners[2].y)},
                {(double)(box->m_corners[3].x), (double)(box->m_corners[3].y)}
            };

            return FDGlobal::rotatePolAroundPol({trianglePolygon}, {hitboxPolygon}, (360.0 - obj->getRotation()) * (M_PI / 180.0));
        } else if (obj->m_objectID == 39 || obj->m_objectID == 178 || obj->m_objectID == 217 || obj->m_objectID == 205) {
            std::vector<CCPointD> triangle = {
                {
                    (double)(obj->m_positionX) - (15.0 * obj->getScaleX()), // lower left
                    (double)(obj->m_positionY) - (15.0 * obj->getScaleY() * 0.5)
                },
                {
                    (double)(obj->m_positionX) + (15.0 * obj->getScaleX()), // lower right
                    (double)(obj->m_positionY) - (15.0 * obj->getScaleY() * 0.5)
                },
                {
                    (double)(obj->m_positionX), // upper center
                    (double)(obj->m_positionY) + (15.0 * obj->getScaleY() * 0.5)
                },
            };

            std::vector<CCPointD> trianglePolygon = {triangle[0], triangle[2], triangle[1]};
            auto box = obj->getOrientedBox();
            std::vector<CCPointD> hitboxPolygon = {
                {(double)(box->m_corners[0].x), (double)(box->m_corners[0].y)},
                {(double)(box->m_corners[1].x), (double)(box->m_corners[1].y)},
                {(double)(box->m_corners[2].x), (double)(box->m_corners[2].y)},
                {(double)(box->m_corners[3].x), (double)(box->m_corners[3].y)}
            };

            return FDGlobal::rotatePolAroundPol({trianglePolygon}, {hitboxPolygon}, (360.0 - obj->getRotation()) * (M_PI / 180.0));
        }
        else if (obj->m_objectID == 392 || obj->m_objectID == 458 || obj->m_objectID == 459) {
            std::vector<CCPointD> triangle = {
                {
                    (double)(obj->m_positionX) - (15.0 * obj->getScaleX() * 0.4), // lower left
                    (double)(obj->m_positionY) - (15.0 * obj->getScaleY() * 0.4)
                },
                {
                    (double)(obj->m_positionX) + (15.0 * obj->getScaleX() * 0.4), // lower right
                    (double)(obj->m_positionY) - (15.0 * obj->getScaleY() * 0.4)
                },
                {
                    (double)(obj->m_positionX), // upper center
                    (double)(obj->m_positionY) + (15.0 * obj->getScaleY() * 0.4)
                },
            };

            std::vector<CCPointD> trianglePolygon = {triangle[0], triangle[2], triangle[1]};
            auto box = obj->getOrientedBox();
            std::vector<CCPointD> hitboxPolygon = {
                {(double)(box->m_corners[0].x), (double)(box->m_corners[0].y)},
                {(double)(box->m_corners[1].x), (double)(box->m_corners[1].y)},
                {(double)(box->m_corners[2].x), (double)(box->m_corners[2].y)},
                {(double)(box->m_corners[3].x), (double)(box->m_corners[3].y)}
            };

            return FDGlobal::rotatePolAroundPol({trianglePolygon}, {hitboxPolygon}, (360.0 - obj->getRotation()) * (M_PI / 180.0));
        } else if (obj->m_objectID == 88) {

        }
        return std::nullopt;
    }

    std::optional<Polygon> getCustomPolygon(PlayerObject *p0) {
        if (p0 == nullptr) return std::nullopt;

        CCPoint playerPos = p0->getPosition();
        float scale = p0->getScale();
        auto ap = p0->getAnchorPoint();

        if (p0->m_isDart) {
            scale *= 0.6f;
        }

        CCPointD ll = {
            (double)(playerPos.x) - (30.0 * ap.x * scale),
            (double)(playerPos.y) - (30.0 * ap.y * scale)
        };
        CCPointD lr = {
            (double)(playerPos.x) + (30.0 * ap.x * scale),
            (double)(playerPos.y) - (30.0 * ap.y * scale)
        };
        CCPointD ul = {
            (double)(playerPos.x) - (30.0 * ap.x * scale),
            (double)(playerPos.y) + (30.0 * ap.y * scale)
        };
        CCPointD ur = {
            (double)(playerPos.x) + (30.0 * ap.x * scale),
            (double)(playerPos.y) + (30.0 * ap.y * scale)
        };

        std::vector<CCPointD> playerPolygon = {ll, ul, ur, lr};
        Polygon res;
        res.points = playerPolygon;

        return FDGlobal::rotatePolygon(res, (360.0 - p0->getRotation()) * (M_PI / 180.0));
    }

    CCPoint convertPointF(const CCPointD &d) {
        return { (float)d.x, (float)d.y };
    }

    std::vector<CCPoint> convertPointsF(const std::vector<CCPointD> &points) {
        std::vector<CCPoint> result;

        for (const auto& p : points) {
            result.push_back(convertPointF(p));
        }

        return result;
    }

    double distanceSquared(const CCPointD& a, const CCPointD& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return dx * dx + dy * dy;
    }
    CCPointD closestPointOnSegment(const CCPointD& a, const CCPointD& b, const CCPointD& p) {
        double ax = b.x - a.x;
        double ay = b.y - a.y;
        double t = ((p.x - a.x) * ax + (p.y - a.y) * ay) / (ax * ax + ay * ay);

        t = fmax(0.0, fmin(1.0, t));

        return {a.x + t * ax, a.y + t * ay};
    }
    bool isPointInPolygon(const Polygon& poly, const CCPointD& point) {
        int n = poly.points.size();
        if (n < 3) return false;

        bool inside = false;
        for (int i = 0, j = n - 1; i < n; j = i++) {
            const CCPointD& p1 = poly.points[i];
            const CCPointD& p2 = poly.points[j];

            if (((p1.y > point.y) != (p2.y > point.y)) &&
                (point.x < (p2.x - p1.x) * (point.y - p1.y) / (p2.y - p1.y) + p1.x)) {
                inside = !inside;
            }
        }
        return inside;
    }
    bool polygonIntersectsCircle(const Polygon& poly, const CCPointD& center, double radius) {
        double radiusSquared = radius * radius;

        for (const CCPointD &p : poly.points) {
            if (distanceSquared(p, center) <= radiusSquared) {
                return true;
            }
        }

        if (isPointInPolygon(poly, center)) {
            return true;
        }

        int n = poly.points.size();
        for (int i = 0, j = n - 1; i < n; j = i++) {
            CCPointD p1 = poly.points[i];
            CCPointD p2 = poly.points[j];

            if (distanceSquared(closestPointOnSegment(p1, p2, center), center) <= radiusSquared) {
                return true;
            }
        }

        return false;
    }

    bool processHitboxesSpecial(GJBaseGameLayer *target, const std::map<PlayerObject*, Polygon> &players, GameObject *hazard, bool display = false) {
        int objectType = 0;
        double radius = 0.f;

        switch (hazard->m_objectID) {
            case 89: case 187: case 184: case 398: case 676: case 679: case 741: case 1706: case 1709: case 1735: {
                radius = 28; // not very small saw
                objectType = 1;
                break;
            }
            case 88: case 186: case 183: case 397: case 675: case 678: case 740: case 1705: case 1708: case 1619: case 1734: {
                radius = 40; // biggest saw
                objectType = 1;
                break;
            }
            case 98: case 185: case 188: case 399: case 677: case 680: case 742: case 1707: case 1710: case 1736: {
                radius = 20; // smallest saw
                objectType = 1;
                break;
            }
            case 1620: {
                radius = 26; // not really a saw imo
                objectType = 1;
                break;
            }
        };

        if (radius != 0) {
            radius *= fmax(hazard->getScaleX(), hazard->getScaleY());
        }

        switch (objectType) {
            case 1: {
                if (display) {
                    FDGlobal::drawNode->drawCircle({(float)hazard->m_positionX, (float)hazard->m_positionY}, (float)radius, {0,0,0,0}, 0.5f, {1,1,0,1}, 32);
                    break;
                }
                for (auto &[k, v] : players) {
                    if (polygonIntersectsCircle(v, {hazard->m_positionX, hazard->m_positionY}, radius)) {
                        // log::info("killed lol");
                        target->destroyPlayer(k, hazard);
                        return true;
                    }
                }
            }
        }

        return false;
    }

    bool processHitboxes(GJBaseGameLayer *target, const std::vector<PlayerObject*> &players, bool display = true) {
        if (
            (   display
                &&
                (
                    !Mod::get()->getSettingValue<bool>("show-real-hitboxes")
                    ||
                    !FDGlobal::drawNode
                )
            )
            ||
            !Mod::get()->getSettingValue<bool>("enable-real-hitboxes")
            ||
            !target
        ) {
            return false;
        }

        if (FDGlobal::drawNode) FDGlobal::drawNode->clear();

        std::map<PlayerObject*, Polygon> playerPolygons = {};

        for (PlayerObject *p0 : players) {
            if (p0 == nullptr) continue;
            auto playerPolygonOpt = FDGlobal::getCustomPolygon(p0);
            if (!playerPolygonOpt.has_value()) continue;
            auto playerPolygon = playerPolygonOpt.value();
            auto fPol = FDGlobal::convertPointsF(playerPolygon.points);
            if (display) FDGlobal::drawNode->drawPolygon(fPol.data(), 4, {0.f, 0.f, 0.f, 0.f}, 0.25f, {1.f, 1.f, 0.f, 1.f});
            playerPolygons[p0] = playerPolygon;
        }

        CCNode *bLayer = target->m_objectLayer;
        if (bLayer == nullptr) {
            log::error("cannot get batch-layer");
        } else {
            CCArray *batches = bLayer->getChildren();
            for (unsigned int c = 0; c < batches->count(); c++) {
                CCNode *objectBatchNode = typeinfo_cast<CCNode*>(batches->objectAtIndex(c));
                if (objectBatchNode == nullptr) {
                    continue;
                }

                CCArray *children = objectBatchNode->getChildren();
                if (children == nullptr) {
                    continue;
                }

                for (unsigned int i = 0; i < children->count(); i++) {
                    GameObject *obj = typeinfo_cast<GameObject*>(children->objectAtIndex(i));
                    if (!obj) {
                        continue;
                    }

                    auto polv = FDGlobal::getCustomPolygon(obj);
                    if (polv.has_value()) {
                        auto pol = polv.value();
                        auto fPol = FDGlobal::convertPointsF(pol.points);
                        if (display) FDGlobal::drawNode->drawPolygon(fPol.data(), fPol.size(), {0,0,0.f}, 0.4f, {1.f, 0.f, 0, 1.f});
                        else {
                            for (const auto &[k, v] : playerPolygons) {
                                if (FDGlobal::IsPolygonsIntersecting(v, pol)) {
                                    target->destroyPlayer(k, obj);
                                    return true;
                                }
                            }
                        }
                    } else {
                        if (processHitboxesSpecial(target, playerPolygons, obj, display)) {
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }
}


class DisappereanceEffectNode : public CCNode {
private:
    std::unordered_map<CCSprite*, double> _elements = {};
    bool _playSound;
    double _mul;
    double _spiralBase = 0;
    double _currentRadius = 3;
    int _spiralStage = 1;
    float _positionScaling = 50.f;
    std::function<void(void)> _onStageTwo = nullptr;


    std::vector<FMOD::Sound*> _loadedSounds = {};
public:
    static FMOD_RESULT nonBlockCallback(FMOD_SOUND *a, FMOD_RESULT b) {
        // log::info("nonBlockCallback called");

        auto engine = FMODAudioEngine::get();
        auto system = engine->m_system;
        FMOD::Channel *playingChannel;

        // system->playSound(, engine->m_globalChannel, 0, &playingChannel);

        return FMOD_OK;
    }

    void playSoundFmod(const std::string &filePath) {
        FMOD_RESULT result;
        FMOD::Sound *sound;
        FMOD_CREATESOUNDEXINFO exinfo;

        memset(&exinfo, 0, sizeof(FMOD_CREATESOUNDEXINFO));
        exinfo.cbsize = sizeof(FMOD_CREATESOUNDEXINFO);
        exinfo.nonblockcallback = DisappereanceEffectNode::nonBlockCallback;

        auto engine = FMODAudioEngine::get();
        auto system = engine->m_system;
        FMOD::Channel *playingChannel;

        if (FDGlobal::grp == nullptr) {
            system->createChannelGroup("funkydash", &FDGlobal::grp);
        }
        FDGlobal::grp->setVolume(GameManager::get()->m_sfxVolume);

        std::string fullpath = Mod::get()->getResourcesDir().string() + "/" + filePath;
        result = system->createStream(fullpath.c_str(), FMOD_DEFAULT | FMOD_LOOP_OFF | FMOD_2D | FMOD_LOWMEM, &exinfo, &sound);

        // log::info("{} {}", filePath, fullpath);

        if (result != FMOD_OK) {
            log::info("FMOD ERROR {}", (int)result);
        } else {
            log::info("fmod: sound created successfully");
            auto res = system->playSound(sound, FDGlobal::grp, false, &playingChannel);
            if (res != FMOD_OK) {
                log::info("FMOD ERROR STARTING AUDIO: {}", (int)res);
            }
            _loadedSounds.push_back(sound);
        }
    }

    static DisappereanceEffectNode *create(std::function<void(void)> onStageTwo = nullptr, bool playSound = false) {
        DisappereanceEffectNode *node = new DisappereanceEffectNode;
        node->init(onStageTwo, playSound);
        node->autorelease();
        return node;
    }

    std::vector<double> getVOrder() {
        return {_spiralBase, _spiralBase + 1.8, _spiralBase + 1.8 + 2.2};
    }

    void removeNode() {
        // FDGlobal::grp->stop();

        // auto engine = FMODAudioEngine::get();
        // auto system = engine->m_system;

        removeMeAndCleanup();
    }
    void beginStage2() {
        _currentRadius = 10;
        _spiralBase = 10;
        _spiralStage = 2;

        if (_playSound) {
            playSoundFmod("snd-hazard-sparkles-ut.wav");
        }

        float initialTime = 2.38f;
        _mul = 1.f / initialTime;

        auto order = getVOrder();
        unsigned int i = 0;
        for (auto &[k, v] : _elements) {
            _elements[k] = order[i];

            CCPoint p = {
                (float)((_currentRadius - _spiralBase) * cos(order[i])) * _positionScaling,
                (float)((_currentRadius - _spiralBase) * sin(order[i])) * _positionScaling
            };

            k->setPosition(p);
            k->runAction(CCFadeTo::create(initialTime, 0));

            i++;
        }

        runAction(CCSequence::create(
            CCDelayTime::create(initialTime),
            CCCallFunc::create(this, callfunc_selector(DisappereanceEffectNode::removeNode)),
            nullptr
        ));

        //
        auto particle = GameToolbox::particleFromString("60a0.5a1.5a0.75a60a48a180a59a0a0a0a0a-1a12a0a166a0a3a1a0a0a1a0a1a0a1a0a1a0a0a0a0a0a1a0a1a0a1a0a1a0a0a0a0.3a0a0a0a36a0a0a0a0a2a1a0a0a0a0a0a0a0a0a0a0a0a0a0a0a2.96a0a0a0", CCParticleSystemQuad::create(), true);
        addChild(particle, -1);

        if (_onStageTwo) {
            _onStageTwo();
        }
    }

    void update(float delta) {
        switch (_spiralStage) {
            case 1: {
                for (auto &[k, v] : _elements) {
                    CCPoint p = {
                        (float)((_currentRadius - _spiralBase) * cos(v)) * _positionScaling,
                        (float)((_currentRadius - _spiralBase) * sin(v)) * _positionScaling
                    };

                    float a = delta * _mul;

                    k->setPosition(p);
                    k->setRotation(k->getRotation() + (a * 20.f));

                    _elements[k] += a;
                    _spiralBase += a;
                }
                break;
            }
            case 2: {
                for (auto &[k, v] : _elements) {
                    CCPoint p = {
                        (float)((_currentRadius - _spiralBase) * cos(v)) * _positionScaling,
                        (float)((_currentRadius - _spiralBase) * sin(v)) * _positionScaling
                    };

                    float a = delta * _mul;

                    k->setPosition(p);
                    k->setRotation(k->getRotation() - (a * 50.f));

                    _elements[k] -= a;
                    _spiralBase -= a;
                }
            }
        }
    }

    bool init(std::function<void(void)> onStageTwo, bool playSound) {
        _playSound = playSound;
        _onStageTwo = onStageTwo;

        auto order = getVOrder();
        for (double o : order) {
            _elements[CCSprite::createWithSpriteFrameName("GJ_deleteIcon_001.png")] = o;
        }

        float initialTime = 0.231f;
        _mul = 1.0 / (double)initialTime;

        for (auto &[k, v] : _elements) {
            k->setScale(0.75f);
            k->setOpacity(0);
            k->setRotation((float)(rand() % 360));
            k->runAction(CCFadeTo::create(initialTime, 255));
            addChild(k);
        }

        runAction(CCSequence::create(
            CCDelayTime::create(0.231f),
            CCCallFunc::create(this, callfunc_selector(DisappereanceEffectNode::beginStage2)),
            nullptr
        ));

        if (_playSound) {
            playSoundFmod("snd-hazard-prepare.wav");
        }

        scheduleUpdate();

        return true;
    }
};

class $modify(XPlayLayer, PlayLayer) {
    struct Fields {
        bool beginShowing = false;
    };

    void hitboxWorkaround(float) {
        log::info("allowing hitbox rendering");
        m_fields->beginShowing = true;
    }

    bool init(GJGameLevel *level, bool p1, bool p2) {
        FDGlobal::hiddenGameObjects.clear();
        FDGlobal::newObjectAmount = 0;
        FDGlobal::oldObjectAmount = 0;

        bool state = PlayLayer::init(level, p1, p2);
        if (!state) return false;

        log::info("creating drawNode");

        FDGlobal::drawNode = CCDrawNode::create();
        FDGlobal::drawNode->setZOrder(1000);
        FDGlobal::drawNode->setID("custom-hitbox-renderer"_spr);

        this->m_objectLayer->addChild(FDGlobal::drawNode);

        scheduleOnce(schedule_selector(XPlayLayer::hitboxWorkaround), 0.5f);

        return true;
    }
    void updateVisibility(float delta) {
        PlayLayer::updateVisibility(delta);
        for (auto &pair : FDGlobal::hiddenGameObjects) {
            pair.first->m_positionX = pair.second.x;
            pair.first->m_positionY = pair.second.y;
        }
        if (FDGlobal::drawNode && m_fields->beginShowing) FDGlobal::processHitboxes(this, {m_player1, m_player2}, true);
    }
    void resetLevel() {
        PlayLayer::resetLevel();
        if (FDGlobal::oldObjectAmount < FDGlobal::newObjectAmount) {
            FMODAudioEngine *engine = FMODAudioEngine::sharedEngine();
            unsigned int delta = FDGlobal::newObjectAmount - FDGlobal::oldObjectAmount;
            log::info("delta is {}", delta);
            if (delta >= 16) {
                log::info("playing effect (>=16)");
                engine->playEffect("snd-hazard-removed-toomuch.mp3"_spr, 1.f, 1.f, 1.5f);
            } else if (delta >= 10) {
                log::info("playing effect (>=10)");
                engine->playEffect("snd-hazard-removed-more.mp3"_spr, 1.f, 1.f, 1.5f);
            } else if (delta >= 5) {
                log::info("playing effect (>=5)");
                engine->playEffect("snd-hazard-removed-few.mp3"_spr, 1.f, 1.f, 1.5f);
            } else if (delta >= 1) {
                log::info("playing effect (>=1)");
                engine->playEffect("snd-hazard-removed-single.mp3"_spr, 1.f, 1.f, 1.5f);
            }
            FDGlobal::oldObjectAmount = FDGlobal::newObjectAmount;
        }
    }
    void destroyPlayer(PlayerObject *p0, GameObject *p1) {
        // log::info("{} {}", p0, p1);
        // FDGlobal::last_collided_obj = p1;
        PlayLayer::destroyPlayer(p0, p1);
    }
};

class $modify(ALevelEditorLayer, LevelEditorLayer){
    bool init(GJGameLevel* level, bool p1) {
        if(!LevelEditorLayer::init(level, p1)) return false;

        log::info("creating drawNode");

        FDGlobal::drawNode = CCDrawNode::create();
        FDGlobal::drawNode->setZOrder(1000);
        FDGlobal::drawNode->setID("custom-hitbox-renderer"_spr);

        this->m_objectLayer->addChild(FDGlobal::drawNode);

        return true;
    }

    void updateDebugDraw() {
        LevelEditorLayer::updateDebugDraw();

        if (m_isDebugDrawEnabled) {
            FDGlobal::processHitboxes(this, {m_player1, m_player2}, true);
        } else {
            FDGlobal::drawNode->clear();
        }
    }
};

class $modify(GJBaseGameLayer) {
    int checkCollisions(PlayerObject *p0, float p1, bool p2) {
        if (!FDGlobal::processHitboxes(this, {m_player1, m_player2}, false)) {
            return GJBaseGameLayer::checkCollisions(p0, p1, p2);
        }
        return 1;
    }
};

class $modify(XPlayerObject, PlayerObject) {
    void applyAlg0() {
        PlayLayer *pl = PlayLayer::get();

        CCNode *bLayer = pl->m_objectLayer;
        if (bLayer == nullptr) {
            log::error("cannot get batch-layer");
            return;
        }

        std::vector<GameObject*> removedObjects = {};

        CCArray *batches = bLayer->getChildren();
        for (unsigned int c = 0; c < batches->count(); c++) {
            CCNode *objectBatchNode = typeinfo_cast<CCNode*>(batches->objectAtIndex(c));
            if (objectBatchNode == nullptr) {
                // log::error("cannot get object batch node at {}", c);
                continue;
            }

            CCArray *children = objectBatchNode->getChildren();
            if (children == nullptr) {
                // log::error("cannot get object list");
                continue;
            }

            // log::info("trying to go through the object list. this array has {} entries", children->count());

            unsigned int to_remove = 0;
            for (unsigned int i = 0; i < children->count(); i++) {
                GameObject *obj = typeinfo_cast<GameObject*>(children->objectAtIndex(i));
                if (!obj) {
                    continue;
                }
                if (obj->getType() == GameObjectType::Hazard || obj->getType() == GameObjectType::AnimatedHazard) {
                    to_remove++;
                }
            }
            to_remove = std::min((unsigned int)Mod::get()->getSettingValue<int>("hazard-amount"), to_remove);
            if (to_remove == 0) {
                log::info("nothing to remove");
                continue;
            }

            unsigned int removed = 0;
            unsigned int attempts = 0;
            bool hidden = removed == to_remove;

            bool delayHide = Mod::get()->getSettingValue<bool>("play-animation") && to_remove <= 500;
            bool shouldPlaySound = true;

            log::info("delayHide = {} && {}", Mod::get()->getSettingValue<bool>("play-animation"), to_remove <= 500);

            while (!hidden && attempts <= 100) {
                unsigned int rng = rand() % children->count();

                GameObject *obj = typeinfo_cast<GameObject*>(children->objectAtIndex(rng));
                if (!obj) {
                    log::error("entry {} from object list is not an object\n", rng);
                    to_remove--;
                    continue;
                }

                if (FDGlobal::objectExists(obj, removedObjects) || FDGlobal::objectExists(obj) ) {
                    attempts++;
                    continue;
                }

                if (obj->getType() == GameObjectType::Hazard || obj->getType() == GameObjectType::AnimatedHazard) {
                    // i hope this would not cause issues later Clueless
                    if (!delayHide) {
                        FDGlobal::hiddenGameObjects.push_back(std::make_pair(obj, ccp(12000, 12000)));
                        removedObjects.push_back(obj);
                    } else {
                        removedObjects.push_back(obj);
                        auto node = DisappereanceEffectNode::create([obj]() {
                            FDGlobal::hiddenGameObjects.push_back(std::make_pair(obj, ccp(12000, 12000)));
                        }, shouldPlaySound);
                        node->setPosition({
                            (float)obj->m_positionX,
                            (float)obj->m_positionY
                        });
                        pl->m_objectLayer->addChild(node, 1000);
                        shouldPlaySound = false;
                    }
                    // log::info("hazard was been hidden");
                    removed++;
                }

                hidden = removed == to_remove;
            }
        }

        if (Mod::get()->getSettingValue<bool>("remove-interacted-hazard") && this->m_collidedObject) {
            log::info("adding collided object too");
            FDGlobal::hiddenGameObjects.push_back(std::make_pair(this->m_collidedObject, ccp(12000, 12000)));
        }
    }
    void applyAlg1() {
        PlayLayer *pl = PlayLayer::get();

        CCNode *bLayer = pl->m_objectLayer;
        if (bLayer == nullptr) {
            log::error("cannot get batch-layer");
            return;
        }

        std::vector<GameObject*> hazards = {};
        std::vector<GameObject*> removedObjects = {};

        CCArray *batches = bLayer->getChildren();
        for (unsigned int c = 0; c < batches->count(); c++) {
            CCNode *objectBatchNode = typeinfo_cast<CCNode*>(batches->objectAtIndex(c));
            if (objectBatchNode == nullptr) {
                // log::error("cannot get object batch node at {}", c);
                continue;
            }

            CCArray *children = objectBatchNode->getChildren();
            if (children == nullptr) {
                // log::error("cannot get object list");
                continue;
            }

            for (unsigned int i = 0; i < children->count(); i++) {
                GameObject *obj = typeinfo_cast<GameObject*>(children->objectAtIndex(i));
                if (!obj) {
                    continue;
                }
                if (obj->getType() == GameObjectType::Hazard || obj->getType() == GameObjectType::AnimatedHazard) {
                    hazards.push_back(obj);
                }
            }
        }

        unsigned int to_remove = std::min((unsigned int)Mod::get()->getSettingValue<int>("hazard-amount"), (unsigned int)hazards.size());
        unsigned int removed = 0;
        unsigned int attempts = 0;
        bool hidden = removed == to_remove;

        bool delayHide = Mod::get()->getSettingValue<bool>("play-animation") && to_remove <= 500;
        bool shouldPlaySound = true;
        bool shouldDestroyBlock = false;

        if (to_remove >= 1) {
            to_remove--;
            shouldDestroyBlock = true;
        }

        log::info("delayHide = {} && {}", Mod::get()->getSettingValue<bool>("play-animation"), to_remove <= 500);

        std::function<void(GameObject*)> fnRemoveObject = [delayHide, &removedObjects, &shouldPlaySound, pl](GameObject *obj) {
            if (!delayHide) {
                FDGlobal::hiddenGameObjects.push_back(std::make_pair(obj, ccp(12000, 12000)));
                removedObjects.push_back(obj);
            } else {
                removedObjects.push_back(obj);
                auto node = DisappereanceEffectNode::create([obj]() {
                    FDGlobal::hiddenGameObjects.push_back(std::make_pair(obj, ccp(12000, 12000)));
                }, shouldPlaySound);
                node->setPosition({
                    (float)obj->m_positionX,
                    (float)obj->m_positionY
                });
                pl->m_objectLayer->addChild(node, 1000);
                shouldPlaySound = false;
            }
        };

        while (!hidden && attempts <= 100) {
            unsigned int rng = rand() % hazards.size();

            GameObject *obj = hazards[rng];

            if (FDGlobal::objectExists(obj, removedObjects) || FDGlobal::objectExists(obj)) {
                attempts++;
                continue;
            }

            fnRemoveObject(obj);

            removed++;
            hidden = removed == to_remove;
        }

        // FDGlobal::last_collided_obj = pl->m_player1->m_collidedObject;
        // log::info("{} {} {}", FDGlobal::last_collided_obj, shouldDestroyBlock, (int)FDGlobal::last_collided_obj->m_objectType);
        // if (shouldDestroyBlock && FDGlobal::last_collided_obj) {
            // fnRemoveObject(FDGlobal::last_collided_obj);
        // }
    }

    void playerDestroyed(bool p0) {
        PlayLayer *pl = PlayLayer::get();
        PlayerObject::playerDestroyed(p0);

        if (!Mod::get()->getSettingValue<bool>("remove-hazards")) {
            return;
        }

        if (!pl) {
            return;
        }

        unsigned int oldAmount = FDGlobal::hiddenGameObjects.size();

        if (!Mod::get()->getSettingValue<bool>("rem-alg")) {
            applyAlg0();
        } else {
            applyAlg1();
        }

        FDGlobal::newObjectAmount = FDGlobal::hiddenGameObjects.size();
        unsigned int delta = FDGlobal::newObjectAmount - oldAmount;

        if (delta <= 0) return;

    };
};

