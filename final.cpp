#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <box2d/box2d.h>
#include <cmath>
#include <vector>
#include <memory>
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <sstream>

// Константы
const int SCREEN_WIDTH = 1380;
const int SCREEN_HEIGHT = 920;
const int FPS = 60;
const float GRAVITY = 9.81f;
const float PIXELS_PER_METER = 30.0f;
const float POLYGON_CLOSING_RADIUS = 15.0f;
const float MAX_BODY_SPEED = 30.0f;
const int PANEL_WIDTH = 250;

// Цвета новой темы
const sf::Color DARK_BACKGROUND(30, 30, 40);
const sf::Color LIGHT_BACKGROUND(50, 50, 60);
const sf::Color ACCENT_COLOR(100, 150, 255);
const sf::Color TEXT_COLOR(240, 240, 240);
const sf::Color BUTTON_COLOR(70, 70, 90);
const sf::Color BUTTON_HOVER_COLOR(90, 90, 110);
const sf::Color BUTTON_ACTIVE_COLOR(120, 120, 140);
const sf::Color RED(255, 100, 100);
const sf::Color GREEN(100, 255, 100);
const sf::Color BLUE(100, 150, 255);
const sf::Color YELLOW(255, 255, 100);
const sf::Color PINK(255, 150, 200, 150);
const sf::Color SLIDER_TRACK_COLOR(60, 60, 80);
const sf::Color SLIDER_HANDLE_COLOR(100, 150, 255);

enum class BodyType {
    CIRCLE,
    RECTANGLE,
    POLYGON,
    SEGMENT
};

enum class SimulationState {
    PAUSED,
    RUNNING,
    EDITING
};

class Button {
public:
    Button(const std::string& textStr, sf::Vector2f position, sf::Vector2f size)
        : rect(size), text(font, textStr, 16) {
        rect.setPosition(position);
        rect.setFillColor(BUTTON_COLOR);
        rect.setOutlineThickness(1);
        rect.setOutlineColor(sf::Color::Transparent);

        // Центрирование текста в кнопке (сдвинуто влево)
        sf::FloatRect textBounds = text.getLocalBounds();
        text.setPosition({
            position.x + 10,  // Фиксированный отступ слева вместо центрирования
            position.y + size.y / 2 - textBounds.size.y / 2 - 5
            });
        text.setFillColor(TEXT_COLOR);
    }

    void draw(sf::RenderWindow& window) {
        window.draw(rect);
        window.draw(text);
    }

    bool contains(sf::Vector2f point) const {
        return rect.getGlobalBounds().contains(point);
    }

    void setState(bool hover, bool active) {
        if (active) {
            rect.setFillColor(BUTTON_ACTIVE_COLOR);
            rect.setOutlineColor(ACCENT_COLOR);
        }
        else if (hover) {
            rect.setFillColor(BUTTON_HOVER_COLOR);
            rect.setOutlineColor(sf::Color::Transparent);
        }
        else {
            rect.setFillColor(BUTTON_COLOR);
            rect.setOutlineColor(sf::Color::Transparent);
        }
    }

    static sf::Font font;

private:
    sf::RectangleShape rect;
    sf::Text text;
};

sf::Font Button::font;

class ModernSlider {
public:
    ModernSlider(float min, float max, float initialValue, sf::Vector2f position, std::string label)
        : minValue(min), maxValue(max), value(initialValue), position(position), label(label), labelText(font, "", 16), valueText(font, "", 16) {

        // Инициализация графических компонентов
        track.setSize({ 180, 6 });
        track.setFillColor(SLIDER_TRACK_COLOR);
        track.setPosition(position);

        handle.setRadius(10);
        handle.setFillColor(SLIDER_HANDLE_COLOR);
        handle.setOutlineColor(TEXT_COLOR);
        handle.setOutlineThickness(1);

        // Настройка текстовых элементов
        labelText.setFont(Button::font);
        labelText.setString(label);
        labelText.setCharacterSize(16);
        labelText.setFillColor(TEXT_COLOR);
        labelText.setPosition({ position.x, position.y - 25 });

        valueText.setFont(Button::font);
        valueText.setCharacterSize(16);
        valueText.setFillColor(TEXT_COLOR);

        // Обновление начальных позиций и значений
        updateHandlePosition();
        updateValueText();
    }

    void draw(sf::RenderWindow& window) {
        window.draw(track);
        window.draw(handle);
        window.draw(labelText);
        window.draw(valueText);
    }

    bool handleEvent(const sf::Event& event, sf::RenderWindow& window) {
        sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));

        // Вариант 1: Использование is() + getIf()
        if (event.is<sf::Event::MouseButtonPressed>()) {
            const auto& mouse = event.getIf<sf::Event::MouseButtonPressed>();
            if (mouse && mouse->button == sf::Mouse::Button::Left) {
                if (handle.getGlobalBounds().contains(mousePos)) {
                    isDragging = true;
                    return true;
                }
            }
        }
        else if (event.is<sf::Event::MouseButtonReleased>()) {
            const auto& mouse = event.getIf<sf::Event::MouseButtonReleased>();
            if (mouse && mouse->button == sf::Mouse::Button::Left) {
                isDragging = false;
            }
        }
        else if (event.is<sf::Event::MouseMoved>() && isDragging) {
            float newX = mousePos.x;
            if (newX < position.x) newX = position.x;
            if (newX > position.x + track.getSize().x) newX = position.x + track.getSize().x;

            float ratio = (newX - position.x) / track.getSize().x;
            value = minValue + ratio * (maxValue - minValue);
            updateHandlePosition();
            updateValueText();
            return true;
        }

        return false;
    }

    float getValue() const { return value; }

private:
    void updateHandlePosition() {
        float ratio = (value - minValue) / (maxValue - minValue);
        handle.setPosition({ position.x + ratio * track.getSize().x - handle.getRadius(),
                           position.y - handle.getRadius() + track.getSize().y / 2 });
    }

    void updateValueText() {
        std::ostringstream oss;
        if (label == "Friction") {
            oss.precision(2);
            oss << std::fixed;
        }
        oss << value;
        valueText.setString(oss.str());
        valueText.setPosition({ position.x + track.getSize().x + 10, position.y - 5 });
    }

    float minValue;
    float maxValue;
    float value;
    sf::Vector2f position;
    std::string label;

    sf::RectangleShape track;
    sf::CircleShape handle;
    bool isDragging = false;
    sf::Font font;
    sf::Text labelText;
    sf::Text valueText;
};

class ToolPanel {
public:
    ToolPanel() {
        background.setSize({ static_cast<float>(PANEL_WIDTH), static_cast<float>(SCREEN_HEIGHT) });
        background.setPosition({ 0, 0 });
        background.setFillColor(DARK_BACKGROUND);
        background.setOutlineColor(sf::Color(80, 80, 90));
        background.setOutlineThickness(2);

        // Создаем кнопки
        buttons.emplace_back("Space - Pause/Run", sf::Vector2f(10, 50), sf::Vector2f(230, 35));
        buttons.emplace_back("C - Add Circle", sf::Vector2f(10, 95), sf::Vector2f(230, 35));
        buttons.emplace_back("R - Add Rectangle", sf::Vector2f(10, 140), sf::Vector2f(230, 35));
        buttons.emplace_back("P - Add Polygon", sf::Vector2f(10, 185), sf::Vector2f(230, 35));
        buttons.emplace_back("S - Add Segment", sf::Vector2f(10, 230), sf::Vector2f(230, 35));

        // Загружаем шрифт
        if (!Button::font.openFromFile("arial.ttf")) {
            std::cerr << "Failed to load font" << std::endl;
        }

        // Создаем разделитель
        separator.setSize({ 180, 1 });
        separator.setPosition({ 10, 330 });
        separator.setFillColor(sf::Color(80, 80, 90));

        // Создаем слайдеры
        massSlider = std::make_unique<ModernSlider>(1.0f, 1000.0f, 50.0f,
            sf::Vector2f(20, 360), "Mass (kg)");
        frictionSlider = std::make_unique<ModernSlider>(0.0f, 1.0f, 0.5f,
            sf::Vector2f(20, 420), "Friction");
    }

    void draw(sf::RenderWindow& window) {
        window.draw(background);

        // Рисуем заголовок
        sf::Text title(Button::font, "Physics Simulator",  22);
        title.setPosition({ 10, 10 });
        title.setFillColor(ACCENT_COLOR);
        window.draw(title);

        // Рисуем кнопки
        for (auto& button : buttons) {
            button.draw(window);
        }

        // Рисуем разделитель
        window.draw(separator);

        // Рисуем слайдеры
        massSlider->draw(window);
        frictionSlider->draw(window);

        // Рисуем подсказки
        sf::Text hints(Button::font, "Controls:\n"
            "LMB: Create/Select\n"
            "RMB: Delete\n"
            "ESC: Cancel", 14);
        hints.setPosition({ 10, SCREEN_HEIGHT - 100 });
        hints.setFillColor(sf::Color(180, 180, 180));
        window.draw(hints);
    }

    int checkHover(sf::Vector2f mousePos) {
        for (size_t i = 0; i < buttons.size(); i++) {
            buttons[i].setState(buttons[i].contains(mousePos), false);
            if (buttons[i].contains(mousePos)) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    bool handleEvent(const sf::Event& event, sf::RenderWindow& window) {
        bool handled = massSlider->handleEvent(event, window);
        if (!handled) {
            handled = frictionSlider->handleEvent(event, window);
        }
        return handled;
    }

    float getMassValue() const { return massSlider->getValue(); }
    float getFrictionValue() const { return frictionSlider->getValue(); }

private:
    sf::RectangleShape background;
    std::vector<Button> buttons;
    sf::RectangleShape separator;
    std::unique_ptr<ModernSlider> massSlider;
    std::unique_ptr<ModernSlider> frictionSlider;
};

class PhysicsBody {
public:
    virtual ~PhysicsBody() = default;
    virtual void draw(sf::RenderWindow& window) const = 0;
    virtual void update() = 0;

    BodyType type;
    b2Body* body = nullptr;
    sf::Vector2f position;
    float rotation = 0.0f;
};

class CircleBody : public PhysicsBody {
public:
    CircleBody(b2World& world, sf::Vector2f pos, float radius, float density, float friction, float restitution) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(pos.x / PIXELS_PER_METER, pos.y / PIXELS_PER_METER);
        bodyDef.linearDamping = 0.05f;
        bodyDef.angularDamping = 0.1f;
        body = world.CreateBody(&bodyDef);

        b2CircleShape circleShape;
        circleShape.m_radius = radius / PIXELS_PER_METER;

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &circleShape;
        fixtureDef.density = density;
        fixtureDef.friction = friction;
        fixtureDef.restitution = restitution;
        body->CreateFixture(&fixtureDef);

        circle.setRadius(radius);
        circle.setOrigin({ radius, radius });
        circle.setPosition(pos);
        circle.setFillColor(RED);
        circle.setOutlineColor(TEXT_COLOR);
        circle.setOutlineThickness(1);

        type = BodyType::CIRCLE;
        position = pos;
    }

    void update() override {
        if (body) {
            b2Vec2 pos = body->GetPosition();
            position = sf::Vector2f(pos.x * PIXELS_PER_METER, pos.y * PIXELS_PER_METER);
            rotation = body->GetAngle() * 180 / b2_pi;
            circle.setPosition(position);
            circle.setRotation(sf::degrees(rotation));
        }
    }

    void draw(sf::RenderWindow& window) const override {
        window.draw(circle);

        float endX = position.x + std::cos(rotation * b2_pi / 180.0f) * circle.getRadius();
        float endY = position.y + std::sin(rotation * b2_pi / 180.0f) * circle.getRadius();

        sf::VertexArray line(sf::PrimitiveType::Lines, 2);
        line[0].position = position;
        line[0].color = TEXT_COLOR;
        line[1].position = { endX, endY };
        line[1].color = TEXT_COLOR;
        window.draw(line);
    }

private:
    sf::CircleShape circle;
};

class RectangleBody : public PhysicsBody {
public:
    RectangleBody(b2World& world, sf::Vector2f pos, sf::Vector2f size, float density, float friction, float restitution) {
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(pos.x / PIXELS_PER_METER, pos.y / PIXELS_PER_METER);
        bodyDef.linearDamping = 0.05f;
        bodyDef.angularDamping = 0.1f;
        body = world.CreateBody(&bodyDef);

        b2PolygonShape boxShape;
        boxShape.SetAsBox(
            (size.x / 2) / PIXELS_PER_METER,
            (size.y / 2) / PIXELS_PER_METER
        );

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &boxShape;
        fixtureDef.density = density;
        fixtureDef.friction = friction;
        fixtureDef.restitution = restitution;
        body->CreateFixture(&fixtureDef);

        rect.setSize(size);
        rect.setOrigin({ size.x / 2, size.y / 2 });
        rect.setPosition(pos);
        rect.setFillColor(GREEN);
        rect.setOutlineColor(TEXT_COLOR);
        rect.setOutlineThickness(1);

        type = BodyType::RECTANGLE;
        position = pos;
    }

    void update() override {
        if (body) {
            b2Vec2 pos = body->GetPosition();
            position = sf::Vector2f(pos.x * PIXELS_PER_METER, pos.y * PIXELS_PER_METER);
            rotation = body->GetAngle() * 180 / b2_pi;
            rect.setPosition(position);
            rect.setRotation(sf::degrees(rotation));
        }
    }

    void draw(sf::RenderWindow& window) const override {
        window.draw(rect);

        float endX = position.x + std::cos(rotation * b2_pi / 180.0f) * 20;
        float endY = position.y + std::sin(rotation * b2_pi / 180.0f) * 20;

        sf::VertexArray line(sf::PrimitiveType::Lines, 2);
        line[0].position = position;
        line[0].color = TEXT_COLOR;
        line[1].position = sf::Vector2f(endX, endY);
        line[1].color = TEXT_COLOR;
        window.draw(line);
    }

private:
    sf::RectangleShape rect;
};

class PolygonBody : public PhysicsBody {
public:
    PolygonBody(b2World& world, const std::vector<sf::Vector2f>& vertices,
        float density, float friction, float restitution) {
        if (vertices.size() < 3) {
            throw std::runtime_error("Polygon must have at least 3 vertices");
        }

        // Упорядочиваем вершины
        std::vector<sf::Vector2f> orderedVertices = vertices;
        this->orderVerticesClockwise(orderedVertices);

        // Рассчитываем центр
        sf::Vector2f center(0, 0);
        for (const auto& v : orderedVertices) center += v;
        center /= static_cast<float>(orderedVertices.size());
        position = center;

        // Создаем тело
        b2BodyDef bodyDef;
        bodyDef.type = b2_dynamicBody;
        bodyDef.position.Set(center.x / PIXELS_PER_METER, center.y / PIXELS_PER_METER);
        bodyDef.linearDamping = 0.1f;
        bodyDef.angularDamping = 0.2f;
        body = world.CreateBody(&bodyDef);

        // Готовим вершины для Box2D
        std::vector<b2Vec2> b2Vertices(orderedVertices.size());
        for (size_t i = 0; i < orderedVertices.size(); i++) {
            sf::Vector2f relativePos = orderedVertices[i] - center;
            b2Vertices[i].Set(relativePos.x / PIXELS_PER_METER, relativePos.y / PIXELS_PER_METER);
        }

        // Создаем форму полигона
        b2PolygonShape polygonShape;
        polygonShape.Set(b2Vertices.data(), static_cast<int32>(b2Vertices.size()));

        // Создаем фикстуру
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &polygonShape;
        fixtureDef.density = density;
        fixtureDef.friction = friction;
        fixtureDef.restitution = restitution;
        body->CreateFixture(&fixtureDef);

        // Настраиваем графическое представление
        polygon.setPointCount(orderedVertices.size());
        for (size_t i = 0; i < orderedVertices.size(); i++) {
            polygon.setPoint(i, orderedVertices[i] - center);
        }
        polygon.setPosition(center);
        polygon.setFillColor(BLUE);
        polygon.setOutlineColor(TEXT_COLOR);
        polygon.setOutlineThickness(1);

        type = BodyType::POLYGON;
    }

    void update() override {
        if (body) {
            b2Vec2 pos = body->GetPosition();
            position = sf::Vector2f(pos.x * PIXELS_PER_METER, pos.y * PIXELS_PER_METER);
            rotation = body->GetAngle() * 180 / b2_pi;
            polygon.setPosition(position);
            polygon.setRotation(sf::degrees(rotation));
        }
    }

    void draw(sf::RenderWindow& window) const override {
        window.draw(polygon);

        float endX = position.x + std::cos(rotation * b2_pi / 180.0f) * 20;
        float endY = position.y + std::sin(rotation * b2_pi / 180.0f) * 20;

        sf::VertexArray line(sf::PrimitiveType::Lines, 2);
        line[0].position = position;
        line[0].color = TEXT_COLOR;
        line[1].position = { endX, endY };
        line[1].color = TEXT_COLOR;
        window.draw(line);
    }

private:
    void orderVerticesClockwise(std::vector<sf::Vector2f>& vertices) {
        sf::Vector2f center(0, 0);
        for (const auto& v : vertices) {
            center += v;
        }
        center /= static_cast<float>(vertices.size());

        std::sort(vertices.begin(), vertices.end(), [&center](const sf::Vector2f& a, const sf::Vector2f& b) {
            return std::atan2(a.y - center.y, a.x - center.x) < std::atan2(b.y - center.y, b.x - center.x);
            });
    }

    sf::ConvexShape polygon;
};

class PhysicsSimulation {
public:
    PhysicsSimulation() : world(b2Vec2(0.0f, GRAVITY)) {
        createWalls();
        createPanelWall();
        state = SimulationState::PAUSED;
        createWalls();
    }
    void createPanelWall() {
    b2BodyDef wallDef;
    wallDef.position.Set(PANEL_WIDTH / PIXELS_PER_METER, 0);
    b2Body* wallBody = world.CreateBody(&wallDef);

    b2EdgeShape wallShape;
    wallShape.Set(
        b2Vec2(0, 0),  // Начальная точка
        b2Vec2(0, SCREEN_HEIGHT / PIXELS_PER_METER)  // Конечная точка
    );

    b2FixtureDef fixtureDef;
    fixtureDef.shape = &wallShape;
    fixtureDef.friction = toolPanel.getFrictionValue();
    fixtureDef.restitution = 0.1f;  // Добавляем упругость
    
    wallBody->CreateFixture(&fixtureDef);
}
    void addPolygon(const std::vector<sf::Vector2f>& vertices) {
        try {
            bodies.push_back(std::make_unique<PolygonBody>(
                world, vertices, toolPanel.getMassValue() / 100.0f,
                toolPanel.getFrictionValue(), 0.2f
            ));

            if (!bodies.empty()) {
                b2Body* body = bodies.back()->body;
                body->SetLinearVelocity(b2Vec2(0, 0));
                body->SetAngularVelocity(0);
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Failed to create polygon: " << e.what() << std::endl;
        }
    }

    void addCircle(sf::Vector2f pos, float radius) {
        bodies.push_back(std::make_unique<CircleBody>(
            world, pos, radius, toolPanel.getMassValue() / 100.0f,
            toolPanel.getFrictionValue(), 0.3f
        ));
    }

    void addRectangle(sf::Vector2f pos, sf::Vector2f size) {
        bodies.push_back(std::make_unique<RectangleBody>(
            world, pos, size, toolPanel.getMassValue() / 100.0f,
            toolPanel.getFrictionValue(), 0.3f
        ));
    }

    void createWalls() {
        b2BodyDef groundBodyDef;
        groundBodyDef.position.Set(0.0f, 0.0f);
        b2Body* groundBody = world.CreateBody(&groundBodyDef);

        b2Vec2 vertices[] = {
            b2Vec2(0, SCREEN_HEIGHT / PIXELS_PER_METER),
            b2Vec2(SCREEN_WIDTH / PIXELS_PER_METER, SCREEN_HEIGHT / PIXELS_PER_METER),
            b2Vec2(SCREEN_WIDTH / PIXELS_PER_METER, 0),
            b2Vec2(0, 0)
        };

        b2ChainShape chain;
        chain.CreateLoop(vertices, 4);
        groundBody->CreateFixture(&chain, 0.0f);
    }

    void addSegment(sf::Vector2f a, sf::Vector2f b) {
        b2BodyDef bodyDef;
        bodyDef.position.Set(0.0f, 0.0f);
        b2Body* body = world.CreateBody(&bodyDef);

        b2ChainShape chain;
        b2Vec2 vertices[2] = {
            b2Vec2(a.x / PIXELS_PER_METER, a.y / PIXELS_PER_METER),
            b2Vec2(b.x / PIXELS_PER_METER, b.y / PIXELS_PER_METER)
        };

        chain.CreateChain(vertices, 2);

        b2FixtureDef fixtureDef;
        fixtureDef.shape = &chain;
        fixtureDef.density = 0.0f;  // Статическое тело
        fixtureDef.friction = toolPanel.getFrictionValue();
        fixtureDef.restitution = 0.3f;
        body->CreateFixture(&fixtureDef);

        segments.push_back(std::make_pair(a, b));
    }

    bool removeBody(sf::Vector2f pos) {
        for (auto it = bodies.begin(); it != bodies.end(); ++it) {
            sf::Vector2f bodyPos = (*it)->position;
            float dx = pos.x - bodyPos.x;
            float dy = pos.y - bodyPos.y;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance < 30.0f) {
                world.DestroyBody((*it)->body);
                bodies.erase(it);
                return true;
            }
        }
        return false;
    }

    bool selectBody(sf::Vector2f pos) {
        for (auto& body : bodies) {
            sf::Vector2f bodyPos = body->position;
            float dx = pos.x - bodyPos.x;
            float dy = pos.y - bodyPos.y;
            float distance = std::sqrt(dx * dx + dy * dy);

            if (distance < 30.0f) {
                selectedBody = body.get();
                forceStart = body->position;
                forceEnd = pos;
                return true;
            }
        }
        selectedBody = nullptr;
        return false;
    }

    void applyForce(sf::Vector2f target) {
        if (selectedBody && selectedBody->body) {
            sf::Vector2f force = forceStart - target;
            float forceScale = 100.0f;

            selectedBody->body->ApplyLinearImpulseToCenter(
                b2Vec2(force.x * forceScale / PIXELS_PER_METER,
                    force.y * forceScale / PIXELS_PER_METER),
                true
            );

            selectedBody = nullptr;
        }
    }

    void update(float dt) {
        if (state == SimulationState::RUNNING) {
            world.Step(dt, 8, 3);

            for (auto& body : bodies) {
                if (body->body) {
                    b2Vec2 velocity = body->body->GetLinearVelocity();
                    float speed = velocity.Length();
                    if (speed > MAX_BODY_SPEED) {
                        velocity *= MAX_BODY_SPEED / speed;
                        body->body->SetLinearVelocity(velocity);
                    }
                    body->update();
                }
            }
        }
    }

    void draw(sf::RenderWindow& window) {
        // Фон рабочей области
        sf::RectangleShape workArea({ SCREEN_WIDTH - PANEL_WIDTH, static_cast<float>(SCREEN_HEIGHT) });
        workArea.setPosition({ static_cast<float>(PANEL_WIDTH), 0 });
        workArea.setFillColor(sf::Color(40, 40, 50));
        window.draw(workArea);

        // Рисуем сегменты
        for (auto segment : segments) {
            sf::VertexArray line(sf::PrimitiveType::Lines, 2);
            line[0].position = segment.first;
            line[0].color = BLUE;
            line[1].position = segment.second;
            line[1].color = BLUE;
            window.draw(line);
        }

        // Рисуем тела
        for (auto& body : bodies) {
            body->draw(window);
        }

        // Рисуем линию силы, если тело выбрано
        if (selectedBody) {
            sf::VertexArray forceLine(sf::PrimitiveType::Lines, 2);
            forceLine[0].position = forceStart;
            forceLine[0].color = YELLOW;
            forceLine[1].position = forceEnd;
            forceLine[1].color = YELLOW;
            window.draw(forceLine);

            sf::CircleShape forceEndCircle(5);
            forceEndCircle.setFillColor(YELLOW);
            forceEndCircle.setOrigin({ 5, 5 });
            forceEndCircle.setPosition(forceEnd);
            window.draw(forceEndCircle);
        }

        // Рисуем полигон в процессе создания
        if (drawingPolygon) {
            if (polygonPoints.size() >= 2) {
                sf::CircleShape closingCircle(POLYGON_CLOSING_RADIUS);
                closingCircle.setOrigin({ POLYGON_CLOSING_RADIUS, POLYGON_CLOSING_RADIUS });
                closingCircle.setPosition(polygonPoints[0]);
                closingCircle.setFillColor(PINK);
                window.draw(closingCircle);
            }

            if (polygonPoints.size() > 1) {
                for (size_t i = 0; i < polygonPoints.size() - 1; i++) {
                    sf::VertexArray line(sf::PrimitiveType::Lines, 2);
                    line[0].position = polygonPoints[i];
                    line[0].color = TEXT_COLOR;
                    line[1].position = polygonPoints[i + 1];
                    line[1].color = TEXT_COLOR;
                    window.draw(line);
                }

                auto mousePos = sf::Mouse::getPosition(window);
                sf::VertexArray previewLine(sf::PrimitiveType::Lines, 2);
                previewLine[0].position = polygonPoints.back();
                previewLine[0].color = sf::Color(255, 255, 255, 150);
                previewLine[1].position = sf::Vector2f(mousePos.x, mousePos.y);
                previewLine[1].color = sf::Color(255, 255, 255, 150);
                window.draw(previewLine);
            }

            for (auto& point : polygonPoints) {
                sf::CircleShape dot(3);
                dot.setFillColor(RED);
                dot.setOrigin({ 3, 3 });
                dot.setPosition(point);
                window.draw(dot);
            }
        }

        // Рисуем превью при создании объектов
        if (resizingObject) {
            auto mousePos = sf::Mouse::getPosition(window);
            sf::Vector2f currentMousePos(mousePos.x, mousePos.y);

            if (currentResizingType == BodyType::CIRCLE) {
                float radius = std::hypot(currentMousePos.x - resizeStartPos.x,
                    currentMousePos.y - resizeStartPos.y);
                sf::CircleShape preview(radius);
                preview.setOrigin({ radius, radius });
                preview.setPosition(resizeStartPos);
                preview.setFillColor(sf::Color(255, 100, 100, 150));
                preview.setOutlineColor(TEXT_COLOR);
                preview.setOutlineThickness(1);
                window.draw(preview);
            }
            else if (currentResizingType == BodyType::RECTANGLE) {
                sf::Vector2f size(
                    std::abs(currentMousePos.x - resizeStartPos.x) * 2,
                    std::abs(currentMousePos.y - resizeStartPos.y) * 2
                );
                sf::RectangleShape preview(size);
                preview.setOrigin({ size.x / 2, size.y / 2 });
                preview.setPosition(resizeStartPos);
                preview.setFillColor(sf::Color(100, 255, 100, 150));
                preview.setOutlineColor(TEXT_COLOR);
                preview.setOutlineThickness(1);
                window.draw(preview);
            }
        }

        // Рисуем панель инструментов
        toolPanel.draw(window);

        // Рисуем информацию о состоянии
        std::string stateText = "State: " + getStateString();
        std::string bodiesText = "Bodies: " + std::to_string(bodies.size());
        std::string fpsText = "FPS: " + std::to_string(static_cast<int>(1.0f / fpsClock.restart().asSeconds()));

        drawText(window, stateText, 300, 10);
        drawText(window, bodiesText, 300, 30);
        drawText(window, fpsText, 300, 50);
    }

    bool handleSliderEvent(const sf::Event& event, sf::RenderWindow& window) {
        return toolPanel.handleEvent(event, window);
    }

private:
    void drawText(sf::RenderWindow& window, const std::string& str, int x, int y) {
        sf::Text text(Button::font, str, 16);
        text.setPosition({ static_cast<float>(x), static_cast<float>(y) });
        text.setFillColor(TEXT_COLOR);
        window.draw(text);
    }

    std::string getStateString() const {
        switch (state) {
        case SimulationState::PAUSED: return "Paused";
        case SimulationState::RUNNING: return "Running";
        case SimulationState::EDITING: return "Editing";
        default: return "Unknown";
        }
    }

public:
    SimulationState state;
    std::vector<std::unique_ptr<PhysicsBody>> bodies;
    std::vector<std::pair<sf::Vector2f, sf::Vector2f>> segments;
    PhysicsBody* selectedBody = nullptr;
    sf::Vector2f forceStart;
    sf::Vector2f forceEnd;
    bool drawingPolygon = false;
    std::vector<sf::Vector2f> polygonPoints;
    bool resizingObject = false;
    sf::Vector2f resizeStartPos;
    BodyType currentResizingType;
    sf::Clock fpsClock;
    ToolPanel toolPanel;

private:
    b2World world;
};

int main() {
    sf::RenderWindow window(sf::VideoMode({ SCREEN_WIDTH, SCREEN_HEIGHT }),
        "Modern Physics Simulation", sf::Style::Titlebar | sf::Style::Close);
    window.setFramerateLimit(FPS);

    PhysicsSimulation simulation;
    sf::Clock deltaClock;

    while (window.isOpen()) {
        float dt = deltaClock.restart().asSeconds();

        sf::Vector2f mousePos = window.mapPixelToCoords(sf::Mouse::getPosition(window));
        int hoveredButton = simulation.toolPanel.checkHover(mousePos);

        for (auto event = window.pollEvent(); event; event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            }
            else if (simulation.handleSliderEvent(*event, window)) {
                continue;
            }
            else if (event->is<sf::Event::KeyPressed>()) {
                auto key = event->getIf<sf::Event::KeyPressed>();

                if (key->scancode == sf::Keyboard::Scan::Space) {
                    simulation.state = (simulation.state == SimulationState::RUNNING) ?
                        SimulationState::PAUSED : SimulationState::RUNNING;
                }
                else if (key->scancode == sf::Keyboard::Scan::C) {
                    auto mousePos = sf::Mouse::getPosition(window);
                    simulation.resizeStartPos = { static_cast<float>(mousePos.x), static_cast<float>(mousePos.y) };
                    simulation.resizingObject = true;
                    simulation.currentResizingType = BodyType::CIRCLE;
                    simulation.state = SimulationState::EDITING;
                }
                else if (key->scancode == sf::Keyboard::Scan::R) {
                    auto mousePos = sf::Mouse::getPosition(window);
                    simulation.resizeStartPos = { static_cast<float>(mousePos.x), static_cast<float>(mousePos.y) };
                    simulation.resizingObject = true;
                    simulation.currentResizingType = BodyType::RECTANGLE;
                    simulation.state = SimulationState::EDITING;
                }
                else if (key->scancode == sf::Keyboard::Scan::P) {
                    simulation.drawingPolygon = true;
                    simulation.polygonPoints.clear();
                    simulation.state = SimulationState::EDITING;
                }
                else if (key->scancode == sf::Keyboard::Scan::S) {
                    if (simulation.polygonPoints.size() < 2) {
                        auto mousePos = sf::Mouse::getPosition(window);
                        simulation.polygonPoints.push_back({ static_cast<float>(mousePos.x), static_cast<float>(mousePos.y) });
                    }
                    else {
                        simulation.addSegment(
                            simulation.polygonPoints[0],
                            simulation.polygonPoints[1]
                        );
                        simulation.polygonPoints.clear();
                        simulation.state = SimulationState::PAUSED;
                    }
                }
                else if (key->scancode == sf::Keyboard::Scan::Escape) {
                    simulation.selectedBody = nullptr;
                    if (simulation.drawingPolygon) {
                        simulation.drawingPolygon = false;
                        simulation.polygonPoints.clear();
                        simulation.state = SimulationState::PAUSED;
                    }
                    if (simulation.resizingObject) {
                        simulation.resizingObject = false;
                        simulation.state = SimulationState::PAUSED;
                    }
                }
            }
            else if (event->is<sf::Event::MouseButtonPressed>()) {
                auto mouse = event->getIf<sf::Event::MouseButtonPressed>();
                sf::Vector2f mousePos(mouse->position.x, mouse->position.y);

                if (mouse->button == sf::Mouse::Button::Left) {
                    if (hoveredButton != -1) {
                        switch (hoveredButton) {
                        case 0: // Pause/Run
                            simulation.state = (simulation.state == SimulationState::RUNNING) ?
                                SimulationState::PAUSED : SimulationState::RUNNING;
                            break;
                        case 1: // Add Circle
                            simulation.resizingObject = true;
                            simulation.currentResizingType = BodyType::CIRCLE;
                            simulation.resizeStartPos = mousePos;
                            simulation.state = SimulationState::EDITING;
                            break;
                        case 2: // Add Rectangle
                            simulation.resizingObject = true;
                            simulation.currentResizingType = BodyType::RECTANGLE;
                            simulation.resizeStartPos = mousePos;
                            simulation.state = SimulationState::EDITING;
                            break;
                        case 3: // Add Polygon
                            simulation.drawingPolygon = true;
                            simulation.polygonPoints.clear();
                            simulation.state = SimulationState::EDITING;
                            break;
                        case 4: // Add Segment
                            if (simulation.polygonPoints.size() < 2) {
                                simulation.polygonPoints.push_back(mousePos);
                            }
                            else {
                                simulation.addSegment(
                                    simulation.polygonPoints[0],
                                    simulation.polygonPoints[1]
                                );
                                simulation.polygonPoints.clear();
                            }
                            break;
                        case 5: // Clear All
                            simulation.bodies.clear();
                            simulation.segments.clear();
                            break;
                        }
                    }
                    else if (simulation.drawingPolygon) {
                        if (simulation.polygonPoints.size() >= 2) {
                            float distance = std::hypot(
                                mousePos.x - simulation.polygonPoints[0].x,
                                mousePos.y - simulation.polygonPoints[0].y
                            );

                            if (distance <= POLYGON_CLOSING_RADIUS) {
                                simulation.addPolygon(simulation.polygonPoints);
                                simulation.drawingPolygon = false;
                                simulation.polygonPoints.clear();
                                simulation.state = SimulationState::PAUSED;
                                continue;
                            }
                        }

                        simulation.polygonPoints.push_back(mousePos);
                    }
                    else if (simulation.selectedBody) {
                        simulation.applyForce(mousePos);
                    }
                    else if (simulation.resizingObject) {
                        float radius = std::hypot(mousePos.x - simulation.resizeStartPos.x,
                            mousePos.y - simulation.resizeStartPos.y);
                        sf::Vector2f size = {
                            std::abs(mousePos.x - simulation.resizeStartPos.x) * 2,
                            std::abs(mousePos.y - simulation.resizeStartPos.y) * 2
                        };

                        if (simulation.currentResizingType == BodyType::CIRCLE) {
                            simulation.addCircle(simulation.resizeStartPos, radius);
                        }
                        else if (simulation.currentResizingType == BodyType::RECTANGLE) {
                            simulation.addRectangle(simulation.resizeStartPos, size);
                        }

                        simulation.resizingObject = false;
                        simulation.state = SimulationState::PAUSED;
                    }
                    else {
                        simulation.selectBody(mousePos);
                    }
                }
                else if (mouse->button == sf::Mouse::Button::Right) {
                    simulation.removeBody(mousePos);
                }
            }
            else if (event->is<sf::Event::MouseMoved>()) {
                auto mouse = event->getIf<sf::Event::MouseMoved>();
                if (simulation.selectedBody) {
                    simulation.forceEnd = sf::Vector2f(mouse->position.x, mouse->position.y);
                }
            }
        }

        simulation.update(dt);
        simulation.draw(window);
        window.display();
    }
    return 0;
}